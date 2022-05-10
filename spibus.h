/**
 * @file spibus.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief SPI Bus driver API.
 * @version 1.0
 * @date 2020-10-12
 *
 * @copyright Copyright (c) 2022
 */

#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <unistd.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>

#define NUM_SPI_MASTER 3 // 0, 1, 2

#ifndef SPIBUS_CS_GPIO
#define SPIBUS_CS_GPIO 0 ///< Set this preprocessor variable to a positive number to support GPIO CS using gpiodev library.
#endif

#define CS_EXTERNAL 0
#define CS_INTERNAL 1

/** @struct spibus
 * @brief spibus structure describes the device that is initialized with {@link spibus_open}.
 * @var spibus::fd
 * SPI File Descriptor
 * @var spibus::xfer
 * SPI Transfer properties (read man page for spi_ioc_transfer for advanced operations.)
 * @var spibus::bus
 * SPI bus device (X in /dev/spidevX.Y)
 * @var spibus::cs
 * SPI bus CS (Y in /dev/spidevX.Y)
 * @var spibus::mode
 * SPI Bus Mode has a value of SPI_MODE_0, SPI_MODE_1, SPI_MODE_2 or SPI_MODE_3 
 * depending on polarity and phase of SPI transactions. Depends on downstream device.
 * @var spibus::lsb
 * LSB first, 0 if MSB first
 * @var spibus::bits
 * Bits per word, usually 8.
 * @var spibus::speed
 * Max speed of the SPI device.
 * @var spibus::cs_internal
 * If this var is set, SPI bus has its own CS, if unset, GPIO needs to be evoked
 * @var spibus::cs_gpio
 * GPIO pin used for chip select. Set this to -1 if SPIBUS_CS_GPIO is not defined.
 * @var spibus::sleeplen
 * Sleep length between two transfers in microseconds.
 * @var spibus::internal_rotation
 * Rotate buffers for MSB internally.
 */
#ifndef _DOXYGEN_
typedef struct __attribute__((packed))
#else
typedef struct
#endif
{
    int fd;
    struct spi_ioc_transfer xfer[1];
    uint8_t bus;
    uint8_t cs;
    uint8_t mode;
    uint8_t lsb;
    uint8_t bits;
    uint32_t speed;
    uint8_t cs_internal;
    int cs_gpio;
    unsigned long sleeplen;
    bool internal_rotation;
} spibus;

/**
 * @brief Opens SPI bus with pre-configured settings to allow
 * communication with SPI device. All the flags in the spibus
 * struct passed to this function must be set as necessary to
 * avoid initialization issues.
 *
 * @param dev spibus struct containing SPI Bus number to open, mode, speed, lsbfirst etc.
 *
 * @return 1 on success, -1 on failure.
 */
int spibus_init(spibus *dev);
/**
 * @brief Write data over SPI bus described by dev.
 *
 * @param dev spibus struct describing the SPI bus.
 * @param data Ideally a buffer of bytes. The function uses the lsb flag to
 * fix the endianness of the input array (assumed to be little endian). Data is written from this buffer during transfer.
 * @param len Length of the supplied buffer (in bytes).
 *
 * @return status of the ioctl call for the transfer.
 */
int spibus_xfer(spibus *dev, void *data, ssize_t len);
/**
 * @brief Transfer data over SPI bus described by dev.
 *
 * @param dev spibus struct describing the SPI bus
 * @param in Ideally a buffer of bytes. The function uses the lsb flag to
 * fix the endianness of the input array (assumed to be little endian). Data is read into the buffer during transfer.
 * @param out Ideally a buffer of bytes. The function uses the lsb flag to
 * fix the endianness of the output array (assumed to be little endian). Data is written from this buffer during transfer.
 * @param len Length of input and output buffers, must be same otherwise memory access violation will occur.
 *
 * @return status of the ioctl call for the transfer.
 */
int spibus_xfer_full(spibus *dev, void *in, void *out, ssize_t len);
/**
 * @brief Invert array for MSB first transfer of multibyte data. Memory
 * management is entirely upon the caller.
 *
 * @param dest Destination pointer.
 * @param src Source pointer.
 * @param len Length of source and destination buffers.
 */
static inline void spibus_invert(void *dest, void *src, ssize_t len)
{
    unsigned int last = len - 1;
    unsigned char *tmpsrc = (unsigned char *)src, *tmpdest = (unsigned char *)dest;
    for (int i = 0; i < len; i++)
    {
        tmpdest[i] = tmpsrc[last - i];
    }
    return;
}
/**
 * @brief Destroy the SPI bus. For errors, look at close() syscall.
 */
void spibus_destroy(spibus *dev);

#endif // SPI_BUS_H
