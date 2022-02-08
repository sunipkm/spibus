/**
 * @file spibus.c
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief SPI Bus driver implementation.
 * @version 1.0
 * @date 2020-10-12
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "spibus.h"
#include <string.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#if SPIBUS_CS_GPIO > 0
#include <gpiodev/gpiodev.h>
#endif

#ifdef eprintf
#undef eprintf
#endif
#define eprintf(str, ...)                                        \
    {                                                            \
        fprintf(stderr, "%s" str "\n", __func__, ##__VA_ARGS__); \
        fflush(stderr);                                          \
    }

static pthread_mutex_t spibus_lock[NUM_SPI_MASTER]; ///< Lock corresponding to a bus master for interleaved read/writes and GPIO issues
static int spibus_mutex_initd = 0;                  ///< mutex initialization indicator

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
    uint8_t mode, lsb, bits;
    uint32_t speed;

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
#if SPIBUS_CS_GPIO > 0
    if (dev->cs_internal == CS_EXTERNAL) // GPIO as chip select
    {
        gpioSetMode(dev->cs_gpio, GPIO_OUT);
        gpioWrite(dev->cs_gpio, GPIO_HIGH);
    }
#endif // SPIBUS_CS_GPIO
    if (dev->speed < 750000)
        speed = 1000000; // Temporary (and max) bus speed
    else if (dev->speed < 2000000)
        speed = 2500000;
    else if (dev->speed < 5000000)
    {
        speed = 8000000;
    }
    else
    {
        eprintf("SPI requested speed %u > 5 MHz, speed unavailable in driver. Make changes and proceed with caution.", dev->speed);
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

    dev->xfer->cs_change = 0;             // Keep CS activated for the whole word
    dev->xfer->delay_usecs = 0;           // delay in microseconds
    dev->xfer->speed_hz = dev->speed;     // speed of communication
    dev->xfer->bits_per_word = dev->bits; // bits per word

    return 1;
}

int spibus_xfer(spibus *dev, void *data, ssize_t len)
{
    int status = 0;
    char *o_data = NULL;
#ifdef SPIDEBUG
    unsigned char *tmp = data;
    fprintf(stderr, "%s: ", __func__);
    for (unsigned i = 0; i < len; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
    if ((!(dev->lsb)) && (dev->internal_rotation) && (len > 1)) // MSB first
    {
        o_data = (char *)malloc(len);
        if (o_data == NULL)
        {
            eprintf("Could not allocate memory for output MSB conversion");
            perror("malloc");
            status = -1;
            goto cleanup;
        }
        spibus_invert(o_data, data, len);
    }
    else
    {
        o_data = data;
    }
    dev->xfer->tx_buf = (uint64_t)o_data;
    dev->xfer->len = len;
    pthread_mutex_lock(&(spibus_lock[dev->bus]));
#if SPIBUS_CS_GPIO > 0
    if (dev->cs_internal == CS_EXTERNAL) // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
    }
#endif // SPIBUS_CS_GPIO
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
#if SPIBUS_CS_GPIO > 0
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
#endif // SPIBUS_CS_GPIO
    pthread_mutex_unlock(&(spibus_lock[dev->bus]));
    if (status < 0)
    {
        perror("SPIBUS: SPI transfer");
        status = -1;
        goto cleanup;
    }
    usleep(dev->sleeplen);
cleanup:
    if ((o_data != NULL) && (o_data != data))
    {
        free(o_data);
        o_data = NULL;
    }
    return status;
}

int spibus_xfer_full(spibus *dev, void *in, void *out, ssize_t len)
{
    int status = 0;
    char *i_data = NULL, *o_data = NULL;
#ifdef SPIDEBUG
    fprintf(stderr, "%s: In -> 0x%p, Out -> 0x%p\n", __func__, in, out);
    unsigned char *tmp = out;
    fprintf(stderr, "%s: Out: ", __func__);
    for (unsigned i = 0; i < ilen; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n");
#endif

    if ((!(dev->lsb)) && (dev->internal_rotation) && (len > 1)) // MSB first
    {
        o_data = (char *)malloc(len);
        if (o_data == NULL)
        {
            eprintf("Could not allocate memory for output MSB conversion");
            perror("malloc");
            status = -1;
            goto cleanup;
        }
        spibus_invert(o_data, out, len);
    }
    else
    {
        o_data = out;
    }

    if ((!(dev->lsb)) && (dev->internal_rotation) && (len > 1)) // MSB first
    {
        i_data = (char *)malloc(len);
        if (i_data == NULL)
        {
            eprintf("Could not allocate memory for input MSB conversion");
            perror("malloc");
            status = -1;
            goto cleanup;
        }
    }
    else
    {
        i_data = in;
    }

    // TODO: 64-bit compatibility in typecasting
    dev->xfer->tx_buf = (uint64_t)o_data;
    dev->xfer->rx_buf = (uint64_t)i_data;
    dev->xfer->len = len; // whichever is shorter to avoid access violation
#ifdef SPIDEBUG
    eprintf("Output length: %d", dev->xfer->len);
#endif
    pthread_mutex_lock(&(spibus_lock[dev->bus]));
#if SPIBUS_CS_GPIO > 0
    if (dev->cs_internal == CS_EXTERNAL) // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
                                           // usleep(100);
    }
#endif // SPIBUS_CS_GPIO
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
#if SPIBUS_CS_GPIO > 0
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
#endif // SPIBUS_CS_GPIO
    pthread_mutex_unlock(&(spibus_lock[dev->bus]));
    if (status < 0)
    {
        perror("SPIBUS: SPI transfer");
        status = -1;
        goto cleanup;
    }
    usleep(dev->sleeplen);

    if ((!(dev->lsb)) && (dev->internal_rotation) && (len > 1)) // MSB first
    {
        spibus_invert(in, i_data, len);
    }
#ifdef SPIDEBUG
    tmp = in;
    fprintf(stderr, "%s: In: ", __func__);
    for (unsigned i = 0; i < olen; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n\n");
    fflush(stderr);
#endif
    status = 1;
cleanup:
    if ((i_data != in) && (i_data != NULL))
    {
        free(i_data);
        i_data = NULL;
    }
    if ((o_data != out) && (o_data != NULL))
    {
        free(o_data);
        o_data = NULL;
    }
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
