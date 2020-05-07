#ifndef SPIRAM_H
#define SPIRAM_H

#include <stdint.h>

uint32_t spi_read(uint32_t address);
void spi_write(uint32_t address, uint32_t data);
void spi_test();

#endif
