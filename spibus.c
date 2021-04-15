#include "spibus.h"
#include <string.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <gpiodev/gpiodev.h>

#define eprintf(str, ...)                                    \
    fprintf(stderr, "%s" str "\n", __func__, ##__VA_ARGS__); \
    fflush(stderr);

static pthread_mutex_t spibus_lock[NUM_SPI_MASTER]; /// Lock corresponding to a bus master for interleaved read/writes and GPIO issues
static int spibus_mutex_initd = 0;                  /// mutex initialization indicator

int spibus_init(spibus *dev)
{
    if (dev == NULL)
    {
        eprintf("Memory not allocated for device");
        return -1;
    }
    memset(dev->xfer, 0x0, sizeof(struct spi_ioc_transfer));
    if (spibus_mutex_initd++ == 0)
    {
        for (int i = 0; i < NUM_SPI_MASTER; i++)
        {
            int ret = pthread_mutex_init(&(spibus_lock[i]), NULL);
            if (ret != 0)
            {
                eprintf("Failed to initialize SPIBUS mutex for master %d. ", i);
                perror("mutex init");
                return ret;
            }
        }
    }
    int file;
    __u8 mode, lsb, bits;
    __u32 speed;

    mode = dev->mode;
    lsb = dev->lsb;
    bits = 0;

    if (dev->bits == 0) // in case bits is zero, means bits was not set by spibus device init
    {
        dev->bits = 8;
    }
    bits = dev->bits;
    if (dev->speed == 0) // in case max speed was not stipulated
    {
        speed = 500000; // 1 MHz
        dev->speed = speed;
    }

    if (dev->cs_internal == CS_EXTERNAL) // GPIO as chip select
    {
        gpioSetMode(dev->cs_gpio, GPIO_OUT);
        gpioWrite(dev->cs_gpio, GPIO_HIGH);
    }
    if (dev->speed < 750000)
        speed = 1000000; // Temporary (and max) bus speed
    else if (dev->speed < 2000000)
        speed = 2500000;
    else
    {
        eprintf("SPI requested speed %u > 2 MHz, speed unavailable in driver. Make changes and proceed with caution.", dev->speed);
        return -1;
    }
    // open SPI bus
    char spibusname[256];
    if (snprintf(spibusname, 256, "/dev/spidev%d.%d", dev->bus, dev->cs) < 0)
    {
        eprintf("Error in creating device bus name");
        return -1;
    }

    if ((file = open(spibusname, O_RDWR)) < 0)
    {
        eprintf("Can not open bus %s", spibusname);
        perror("Error: ");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_MODE, &mode) < 0)
    {
        perror("SPIBUS: SPI write mode");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_MODE, &mode) < 0)
    {
        perror("SPIBUS: SPI read mode");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_LSB_FIRST, &lsb) < 0)
    {
        perror("SPIBUS: SPI write lsb mode");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
    {
        perror("SPIBUS: SPI read lsb mode");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
        perror("SPIBUS: SPI write bits per word");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
    {
        perror("SPIBUS: SPI read bits per word");
        return -1;
    }

    if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        perror("SPIBUS: SPI write speed hz");
        return -1;
    }

    if (ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
    {
        perror("SPIBUS: SPI read speed hz");
        return -1;
    }

    dev->fd = file; // Update the file descriptor

    dev->xfer[0].cs_change = 0;             // Keep CS activated for the whole word
    dev->xfer[0].delay_usecs = 0;           // delay in microseconds
    dev->xfer[0].speed_hz = dev->speed;     // speed of communication
    dev->xfer[0].bits_per_word = dev->bits; // bits per word

    return 1;
}

/**
 * @brief Invert array for MSB first transfer of multibyte data. Memory
 * management is entirely upon the caller.
 * 
 * @param dest Destination pointer
 * @param src Source pointer
 * @param len Length of source and destination buffers.
 */
inline void spibus_invert(void *dest, void *src, ssize_t len)
{
    unsigned int last = len - 1;
    unsigned char *tmpsrc = (unsigned char *)src, *tmpdest = (unsigned char *)dest;
    for (int i = 0; i < len; i++)
    {
        tmpdest[i] = tmpsrc[last - i];
    }
    return;
}

int spibus_xfer(spibus *dev, void *data, ssize_t len)
{
    int status = 0;
#ifdef SPIDEBUG
    unsigned char *tmp = data;
    fprintf(stderr, "%s: ", __func__);
    for (unsigned i = 0; i < len; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
    char *o_data = (char *)malloc(len);
    if (o_data == NULL)
    {
        eprintf("Could not allocate memory for MSB conversion");
        perror("malloc");
        return -1;
    }
    if (!dev->lsb) // MSB first
    {
        spibus_invert(o_data, data, len);
    }
    else
    {
        memcpy(o_data, data, len);
    }
    // TODO: 64-bit compatibility in typecasting
    dev->xfer[0].tx_buf = (unsigned long)o_data;
    dev->xfer[0].len = len;
    pthread_mutex_lock(&(spibus_lock[dev->bus]));
    if (dev->cs_internal == CS_EXTERNAL) // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
    }
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
    pthread_mutex_unlock(&(spibus_lock[dev->bus]));
    if (status < 0)
    {
        perror("SPIBUS: SPI transfer");
        return -1;
    }
    usleep(dev->sleeplen);
    return status;
}

int spibus_xfer_full(spibus *dev, void *in, ssize_t ilen, void *out, ssize_t olen)
{
    int status = 0;
#ifdef SPIDEBUG
    fprintf(stderr, "%s: In -> 0x%p, Out -> 0x%p\n", __func__, in, out);
    unsigned char *tmp = out;
    fprintf(stderr, "%s: Out: ", __func__);
    for (unsigned i = 0; i < ilen; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n");
#endif
    ssize_t len = ilen < olen ? ilen : olen; // common length

    char *o_data = (char *)malloc(len);
    if (!dev->lsb) // MSB first
    {
        spibus_invert(o_data, out, len);
    }
    else
    {
        memcpy(o_data, out, len);
    }

    char *i_data = (char *)malloc(len);
    if (i_data == NULL)
    {
        eprintf("Could not allocate memory for MSB conversion");
        perror("malloc");
        return -1;
    }
    // TODO: 64-bit compatibility in typecasting
    dev->xfer[0].tx_buf = (unsigned long)o_data;
    dev->xfer[0].rx_buf = (unsigned long)i_data;
    dev->xfer[0].len = len; // whichever is shorter to avoid access violation
#ifdef SPIDEBUG
    eprintf("Output length: %d", dev->xfer[0].len);
#endif
    pthread_mutex_lock(&(spibus_lock[dev->bus]));
    if (dev->cs_internal == CS_EXTERNAL) // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
                                           // usleep(100);
    }
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
    pthread_mutex_unlock(&(spibus_lock[dev->bus]));
    if (status < 0)
    {
        perror("SPIBUS: SPI transfer");
        return -1;
    }
    usleep(dev->sleeplen);

    if (!dev->lsb) // MSB first
    {
        spibus_invert(in, i_data, len);
    }
    else
    {
        memcpy(in, i_data, len);
    }
#ifdef SPIDEBUG
    tmp = in;
    fprintf(stderr, "%s: In: ", __func__);
    for (unsigned i = 0; i < olen; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
    return status;
}

void spibus_destroy(spibus *dev)
{
    if (--spibus_mutex_initd == 0)
    {
        for (int i = 0; i < NUM_SPI_MASTER; i++)
        {
            int ret = pthread_mutex_destroy(&(spibus_lock[i]));
            if (ret != 0)
            {
                eprintf("Failed to destroy SPIBUS mutex for master %d. ", i);
                perror("mutex destroy");
            }
        }
    }
    close(dev->fd);
}
