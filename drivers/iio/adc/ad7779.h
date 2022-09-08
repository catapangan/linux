/*
 * AD7779 Header
 *
 * Copyright 2016-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#define AD7779_SPI_READ_CMD	0x80
#define AD7779_SPI_WRITE_CMD	0x00

#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
// struct ad7779_phy {
// 	struct spi_device *spi;

// };

// int ad7779_spi_read(struct spi_device *spi, unsigned char *rbuf, unsigned reg);
// int ad7779_spi_write(struct spi_device *spi, unsigned val, unsigned reg);
