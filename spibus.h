/**
 * @file spibus.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief SPI Bus driver data structures and function prototypes for SPACE HAUC
 * @version 0.1
 * @date 2020-10-12
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef SPI_BUS_H
#define SPI_BUS_H

#include <stdint.h>
#include <sys/types.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define CS_INTERNAL 1
#define CS_EXTERNAL 0

typedef struct __attribute__((packed))
{
    int fd;                          /// SPI File Descriptor
    struct spi_ioc_transfer xfer[1]; /// SPI Transfer IO buffer
    uint8_t bus;                     /// SPI bus device (X in /dev/spidevX.Y)
    uint8_t cs;                      /// SPI bus CS (Y in /dev/spidevX.Y)
    uint8_t mode;                    /// SPI Mode (SPI_MODE0 etc)
    uint8_t lsb;                     /// LSB first, 0 if MSB first
    uint8_t bits;                    /// Bits per word, usually 8
    uint32_t speed;                  /// Max speed of device
    uint8_t cs_internal;             /// If this var is set, SPI bus has its own CS, if unset, GPIO needs to be evoked
    int cs_gpio;                     /// GPIO chipselect
    unsigned long sleeplen;          /// Sleep length between two transfers in microseconds
} spibus;

/**
 * @brief Initializes SPI bus with pre-configured settings. Settings are preconfigured
 * by device initializer whose member is the spibus struct that is passed as input.
 * 
 * @param dev spibus struct containing SPI Bus number to open, mode, speed, lsbfirst etc
 * 
 * @return 1 on success, -1 on failure.
 */
int spibus_init(spibus *dev);
/**
 * @brief Transfer data over SPI bus described by dev.
 * 
 * @param dev spibus struct describing the SPI bus
 * @param data Ideally a buffer of bytes. The transfer function will not flip
 * little endian data to meet MSBFirst data transfers, though the spibus library
 * provides a function for the inversion.
 * @param len Length of the supplied buffer (in bytes)
 * 
 * @return status of the ioctl call for the transfer
 */
int spibus_xfer(spibus *dev, void *data, ssize_t len);
/**
 * @brief Transfer data over SPI bus described by dev.
 * 
 * @param dev spibus struct describing the SPI bus
 * @param in Ideally a buffer of bytes. The transfer function will not flip
 * little endian data to meet MSBFirst data transfers, though the spibus library
 * provides a function for the inversion.
 * @param ilen Length of the supplied buffer (in bytes)
 * @param out Pointer to output data buffer (bytes)
 * @param olen Length of expected output (bytes)
 * 
 * @return status of the ioctl call for the transfer
 */
int spibus_xfer_full(spibus *dev, void *in, ssize_t ilen, void *out, ssize_t olen);
/**
 * @brief Destroy the SPI bus. For errors, look at close() syscall.
 */
void spibus_destroy(spibus *dev);
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
    unsigned char *tmpsrc = (unsigned char *) src, *tmpdest = (unsigned char *) dest;
    for (int i = 0; i < len; i++)
    {
        tmpdest[i] = tmpsrc[last - i];
    }
    return;
}

#endif // SPI_BUS_H