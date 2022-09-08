/*
 * AD7779 ADC
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#include "ad7779.h"

struct ad7779_state {
	struct spi_device 	*spi;
	
	unsigned int 		crc_enabled;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_rx_buf[2] ____cacheline_aligned;
	u8			reg_tx_buf[2];
};

static int ad7779_spi_read(struct iio_dev *indio_dev, u8 *rbuf, u8 reg) {
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int ret;

	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
			.rx_buf = spi_st->reg_rx_buf,
			.len = 2,
		},
	};

	spi_st->reg_tx_buf[0] = AD7779_SPI_READ_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = 0;

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, ARRAY_SIZE(reg_read_tr));
	
	*rbuf = spi_st->reg_rx_buf[1];

	return ret;
} 

int ad7779_spi_write(struct iio_dev *indio_dev, u8 val, u8 reg) {
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = spi_st->reg_tx_buf,
			.len = 2,
		},
	};

	spi_st->reg_tx_buf[0] = AD7779_SPI_WRITE_CMD | (reg & 0x7F);
	spi_st->reg_tx_buf[1] = val;

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, ARRAY_SIZE(reg_read_tr));
	
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

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad7779_spi_read(indio_dev, &temp, AD7779_REG_CH_CONFIG(chan->channel));
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
		//ret =  ad7779_soft_reset(indio_dev);
		ret = ad7779_spi_write(indio_dev, val, AD7779_REG_CH_CONFIG(chan->channel));
		if(ret)
			return ret;
		return 0;
	}

	return -EINVAL;
}

static int ad7779_soft_reset(struct iio_dev *indio_dev)
{
	struct ad7779_state *spi_st = iio_priv(indio_dev);
	u8 reset_buf[64] = { 0xFF };
	int ret;
	struct spi_transfer reg_read_tr[] = {
		{
			.tx_buf = reset_buf,
			.len = 64,
		},
	};

	ret = spi_sync_transfer(spi_st->spi, reg_read_tr, ARRAY_SIZE(reg_read_tr));

	return ret;
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
};

static int ad7779_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;
	struct ad7779_state *ad7779_st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad7779_st));
	if(!indio_dev)
		return -ENOMEM;
	
	indio_dev->name = "ad7779";
	indio_dev->info = &ad7779_info;
	indio_dev->channels = ad7779_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7779_channels);
	
	ad7779_st = iio_priv(indio_dev);
	ad7779_st->spi = spi;
	ad7779_st->crc_enabled = false;
	
	//ad7779_soft_reset(spi);
	// u8 val_buf[2];
	// val_buf[0] = 0x3E;
	// val_buf[1] = 0;
	// ad7779_spi_write(indio_dev, val_buf, AD7779_REG_GEN_ERR_REG_1_EN);

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
