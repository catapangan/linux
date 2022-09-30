/*
 * AD7779 ADC
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/crc8.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define AD7779_SPI_READ_CMD	0x80
#define AD7779_SPI_WRITE_CMD	0x00

#define AD7779_ENABLE_CRC	0x3F
#define AD7779_ENABLE_SAR	0x29
#define AD7779_DISABLE_SAR	0x09
#define AD7779_PDB_SAR		0x2C
#define AD7779_PDB_SAR_OFF	0x24
#define AD7779_ENABLE_SD	0x90

#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable

#define AD7779_CRC8_POLY	0x07
DECLARE_CRC8_TABLE(ad7779_crc8_table);

struct ad7779_state {
	struct spi_device 	*spi;
	
	unsigned int 		crc_enabled;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_rx_buf[3] ____cacheline_aligned;
	u8			reg_tx_buf[3];
};

static int ad7779_spi_read(struct iio_dev *indio_dev, u8 *rbuf, u8 reg) {
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int ret;
	int length = 2;
	u8 crc_buf[2];
	u8 exp_crc = 0;
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
			.rx_buf = spi_st->reg_rx_buf,
		},
	};

	if(spi_st->crc_enabled)
		length = 3;
	reg_read_tr[0].len = length;	

	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table, 
					spi_st->reg_tx_buf, 2, 0);

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, 
				ARRAY_SIZE(reg_read_tr));
	
	crc_buf[0] = AD7779_SPI_READ_CMD | (reg & 0x7F);
	crc_buf[1] = spi_st->reg_rx_buf[1];
	exp_crc = crc8(ad7779_crc8_table, crc_buf, 2, 0);
	if(spi_st->crc_enabled && (exp_crc != spi_st->reg_rx_buf[2])) {
		dev_err(&spi_st->spi->dev, "Bad CRC %x, expected %x",
			spi_st->reg_rx_buf[2], exp_crc);
		return -EINVAL;
	}
	*rbuf = spi_st->reg_rx_buf[1];

	return ret;
} 

int ad7779_spi_write(struct iio_dev *indio_dev, u8 reg, u8 val) {
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int length = 2;
	struct spi_transfer reg_write_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
		},
	};


	if(spi_st->crc_enabled)
		length = 3;
	reg_write_tr[0].len = length;

	spi_st->reg_tx_buf[0] = AD7779_SPI_WRITE_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = val;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table,
					spi_st->reg_tx_buf, 2, 0);
	
	return spi_sync_transfer(spi_st->spi, reg_write_tr,
				ARRAY_SIZE(reg_write_tr));
}

static int ad7779_spi_sar_mode_en(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_2,
				AD7779_ENABLE_SAR);
	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_1,
				AD7779_PDB_SAR);

	return ret;
}

static int ad7779_spi_sar_mode_disable(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_2,
				AD7779_DISABLE_SAR);
	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_1,
				AD7779_PDB_SAR_OFF);

	return ret;
}

static int ad7779_spi_sar_readback(struct iio_dev *indio_dev, int *readback) {
	int ret;
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int length = 2;
	struct spi_transfer sar_readback_tr[] = {
		{
			.rx_buf = spi_st->reg_rx_buf,
			.tx_buf = spi_st->reg_tx_buf,
		}
	};

	if (spi_st->crc_enabled)
		length = 3;
	sar_readback_tr[0].len = length;
	
	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD;
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = crc8(ad7779_crc8_table,
					spi_st->reg_tx_buf, 2, 0);

	ret = spi_sync_transfer(spi_st->spi, sar_readback_tr,
				ARRAY_SIZE(sar_readback_tr));

	*readback = ((0x0F & spi_st->reg_rx_buf[0]) << 8)
			| spi_st->reg_rx_buf[1];

	return ret;
}

static int ad7779_data_read_en(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_3,
				AD7779_ENABLE_SD);

	return ret;
}

static int ad7779_sigma_delta_data(struct iio_dev *indio_dev) {
	int ret;
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int length = 2;
	// u8 tbuf[8] = { 0x80 };
	// u8 rbuf[8];
	struct spi_transfer sd_readback_tr[] = {
		{
			.rx_buf = spi_st->reg_rx_buf,
			.tx_buf = spi_st->reg_tx_buf,
			.len = 3,
		}
	};

	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD;
	spi_st->reg_tx_buf[1] = 0;
	spi_st->reg_tx_buf[2] = 0;

	ret = spi_sync_transfer(spi_st->spi, sd_readback_tr,
				ARRAY_SIZE(sd_readback_tr));

	dev_info(&spi_st->spi->dev, "The data gathered is %x %x %x ", spi_st->reg_rx_buf[0], spi_st->reg_rx_buf[1], spi_st->reg_rx_buf[2]);

	return ret;
}

static int ad7779_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask) 
{
	int ret;
	u8 temp;
	int data;

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		// ret = ad7779_spi_read(indio_dev, &temp,
		// 			AD7779_REG_CH_CONFIG(chan->channel));
		// ret = ad7779_spi_sar_diagnostic(indio_dev, &data);
		ret = ad7779_sigma_delta_data(indio_dev);
		if(ret)
			return ret;
		*val = temp;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad7779_write_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int val,
			int val2,
			long mask) 
{
	int ret;

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad7779_spi_write(indio_dev,
					AD7779_REG_CH_CONFIG(chan->channel),
					val);
		if(ret)
			return ret;
		return 0;
	}

	return -EINVAL;
}

static int ad7779_soft_reset(struct iio_dev *indio_dev)
{
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	u8 reset_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = &reset_buf[0],
			.len = 8,
		},
	};

	return spi_sync_transfer(spi_st->spi, reg_read_tr,
				ARRAY_SIZE(reg_read_tr));
}

static const struct iio_info ad7779_info = {
	.read_raw = ad7779_read_raw,
	.write_raw = ad7779_write_raw,
};

static const struct iio_chan_spec ad7779_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 0,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 1,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 2,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 3,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 4,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 5,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 6,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 0,
		.indexed = 1,
		.channel = 7,
	},
};

static int ad7779_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;
	struct ad7779_state *ad7779_st;
	u8 temp, temp2;
	int rbuf;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad7779_st));
	if(!indio_dev)
		return -ENOMEM;
	
	indio_dev->name = "ad7779";
	indio_dev->info = &ad7779_info;
	indio_dev->channels = ad7779_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7779_channels);
	
	ad7779_st = iio_priv(indio_dev);
	ad7779_st->spi = spi;

	crc8_populate_msb(ad7779_crc8_table, AD7779_CRC8_POLY);

	/* write 0x3F to REG_GEN_ERR_1 to enable crc */
	ad7779_st->crc_enabled = true;
	ad7779_spi_write(indio_dev,
			AD7779_REG_GEN_ERR_REG_1_EN,
			0x3E);
	ad7779_st->crc_enabled = false;

	ad7779_data_read_en(indio_dev);

	// ad7779_spi_sar_diagnostic(indio_dev, &rbuf);
	// dev_info(&spi->dev, "The data readback is %d", rbuf);
	//ad7779_soft_reset(indio_dev);

	// ad7779_spi_read(indio_dev, &temp2, AD7779_REG_GEN_ERR_REG_1);
	// dev_info(&spi->dev, "The value of general err reg 1 is %x", temp2);

	// ad7779_spi_read(indio_dev, &temp2, AD7779_REG_GEN_ERR_REG_2);
	// dev_info(&spi->dev, "The value of general err reg 2 is %x", temp2);


	// ad7779_spi_read(indio_dev, &temp2, AD7779_REG_GENERAL_USER_CONFIG_1);
	// dev_info(&spi->dev, "The value of general USER CONFIG is %x", temp2);


	// ad7779_spi_read(indio_dev, &temp2, AD7779_REG_GENERAL_USER_CONFIG_2);
	// dev_info(&spi->dev, "The value of general user config 2 is %x", temp2);

	// ad7779_spi_read(indio_dev, &temp, AD7779_REG_GEN_ERR_REG_1_EN);
	// dev_info(&spi->dev, "The value of general err reg en is %x", temp);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad7779_driver = {
	.driver = {
		.name = "ad7779",
	},
	.probe = ad7779_probe,
};
module_spi_driver(ad7779_driver);

MODULE_AUTHOR("Ramona Alexandra Nechita <ramona.nechita@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7779 ADC");
MODULE_LICENSE("GPL v2");
