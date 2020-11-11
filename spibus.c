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

int spibus_init(spibus *dev)
{
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
        speed = 1000000; // 1 MHz
        dev->speed = speed;
    }

    if (dev->cs_internal == CS_EXTERNAL) // GPIO as chip select
    {
        gpioSetMode(dev->cs_gpio, GPIO_OUT);
        gpioWrite(dev->cs_gpio, GPIO_HIGH);
    }
	speed = 2500000;
    // open SPI bus
    char spibusname[256];
    if (snprintf(spibusname, 256, "/dev/spidev%d.%d", dev->bus, dev->cs) < 0)
    {
        fprintf(stderr, "%s: Error in creating device bus name\n", __func__);
        return -1;
    }

    if ((file = open(spibusname, O_RDWR)) < 0)
    {
        fprintf(stderr, "SPIBUS: Opening bus %s", spibusname);
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
    dev->xfer[0].speed_hz = speed;          // speed of communication
    dev->xfer[0].bits_per_word = dev->bits; // bits per word

    return 1;
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
#endif
    dev->xfer[0].tx_buf = (unsigned long)data;
    dev->xfer[0].len = len;
    if (dev->cs_internal == CS_EXTERNAL) // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
    }
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
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
    dev->xfer[0].tx_buf = (unsigned long)out;
    dev->xfer[0].rx_buf = (unsigned long)in;
    dev->xfer[0].len = ilen < olen ? ilen : olen; // whichever is shorter to avoid access violation
#ifdef SPIDEBUG
	fprintf(stderr, "%s: Output length: %d\n", __func__, dev->xfer[0].len);
#endif
    if (dev->cs_internal == CS_EXTERNAL)          // chip select is not internal
    {
        gpioWrite(dev->cs_gpio, GPIO_LOW); // active low transfer
		// usleep(100);
    }
    status = ioctl(dev->fd, SPI_IOC_MESSAGE(1), dev->xfer);
    if (dev->cs_internal == CS_EXTERNAL)
    {
        gpioWrite(dev->cs_gpio, GPIO_HIGH); // high after transfer
    }
    if (status < 0)
    {
        perror("SPIBUS: SPI transfer");
        return -1;
    }
#ifdef SPIDEBUG
    tmp = in;
    fprintf(stderr, "%s: In: ", __func__);
    for (unsigned i = 0; i < olen; i++)
        fprintf(stderr, "%02X ", tmp[i]);
    fprintf(stderr, "\n\n");
#endif
    usleep(dev->sleeplen);
    return status;
}

void spibus_destroy(spibus *dev)
{
    close(dev->fd);
}
