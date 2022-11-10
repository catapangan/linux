/*
 * AD7779 ADC
 *
 * Copyright 2022 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#define AD7779_SPI_READ_CMD	0x80
#define AD7779_SPI_WRITE_CMD	0x00

#define AD7779_ENABLE_CRC	0x3F
#define AD7779_ENABLE_SAR	0x29
#define AD7779_DISABLE_SAR	0x09
#define AD7779_PDB_SAR		0x2C
#define AD7779_PDB_SAR_OFF	0x24
#define AD7779_ENABLE_SD	0x80
#define AD7779_DISABLE_SD	0x80


#define AD7779_REG_CH_CONFIG(ch)		(0x00 + (ch))		// Channel Configuration
#define AD7779_REG_CH_DISABLE			0x08			// Disable clocks to ADC channel
#define AD7779_REG_CH_SYNC_OFFSET(ch)		(0x09 + (ch))		// Channel SYNC Offset
#define AD7779_REG_GENERAL_USER_CONFIG_1	0x11			// General User Config 1
#define AD7779_REG_GENERAL_USER_CONFIG_2	0x12			// General User Config 2
#define AD7779_REG_GENERAL_USER_CONFIG_3	0x13			// General User Config 3
#define AD7779_REG_DOUT_FORMAT			0x14			// Data out format
#define AD7779_REG_ADC_MUX_CONFIG		0x15			// Main ADC meter and reference Mux control
#define AD7779_REG_GLOBAL_MUX_CONFIG		0x16			// Global diagnostics mux
#define AD7779_REG_GPIO_CONFIG			0x17			// GPIO config
#define AD7779_REG_GPIO_DATA			0x18			// GPIO Data
#define AD7779_REG_BUFFER_CONFIG_1		0x19			// Buffer Config 1
#define AD7779_REG_BUFFER_CONFIG_2		0x1A			// Buffer Config 2
#define AD7779_REG_CH_OFFSET_UPPER_BYTE(ch)	(0x1C + (ch) * 6)	// Channel offset upper byte
#define AD7779_REG_CH_OFFSET_MID_BYTE(ch)	(0x1D + (ch) * 6)	// Channel offset middle byte
#define AD7779_REG_CH_OFFSET_LOWER_BYTE(ch)	(0x1E + (ch) * 6)	// Channel offset lower byte
#define AD7779_REG_CH_GAIN_UPPER_BYTE(ch)	(0x1F + (ch) * 6)	// Channel gain upper byte
#define AD7779_REG_CH_GAIN_MID_BYTE(ch)		(0x20 + (ch) * 6)	// Channel gain middle byte
#define AD7779_REG_CH_GAIN_LOWER_BYTE(ch)	(0x21 + (ch) * 6)	// Channel gain lower byte
#define AD7779_REG_CH_ERR_REG(ch)		(0x4C + (ch))		// Channel Status Register
#define AD7779_REG_CH0_1_SAT_ERR		0x54			// Channel 0/1 DSP errors
#define AD7779_REG_CH2_3_SAT_ERR		0x55			// Channel 2/3 DSP errors
#define AD7779_REG_CH4_5_SAT_ERR		0x56			// Channel 4/5 DSP errors
#define AD7779_REG_CH6_7_SAT_ERR		0x57			// Channel 6/7 DSP errors
#define AD7779_REG_CHX_ERR_REG_EN		0x58			// Channel 0-7 Error Reg Enable
#define AD7779_REG_GEN_ERR_REG_1		0x59			// General Errors Register 1
#define AD7779_REG_GEN_ERR_REG_1_EN		0x5A			// General Errors Register 1 Enable
#define AD7779_REG_GEN_ERR_REG_2		0x5B			// General Errors Register 2
#define AD7779_REG_GEN_ERR_REG_2_EN		0x5C			// General Errors Register 2 Enable
#define AD7779_REG_STATUS_REG_1			0x5D			// Error Status Register 1
#define AD7779_REG_STATUS_REG_2			0x5E			// Error Status Register 2
#define AD7779_REG_STATUS_REG_3			0x5F			// Error Status Register 3
#define AD7779_REG_SRC_N_MSB			0x60			// Decimation Rate (N) MSB
#define AD7779_REG_SRC_N_LSB			0x61			// Decimation Rate (N) LSB
#define AD7779_REG_SRC_IF_MSB			0x62			// Decimation Rate (IF) MSB
#define AD7779_REG_SRC_IF_LSB			0x63			// Decimation Rate (IF) LSB
#define AD7779_REG_SRC_UPDATE			0x64			// SRC load source and load update

#define AD777X_POWER_MODE_HIGH			(1 << 6)
#define AD777X_POWER_MODE_LOW			(0 << 6)

#define AD7779_LOW_POWER_MODE			0
#define AD7779_HIGH_POWER_MODE			1


#define AD7779_CRC8_POLY	0x07
DECLARE_CRC8_TABLE(ad7779_crc8_table);

struct ad7779_state {
	struct spi_device 	*spi;
	struct clk 		*mclk;
	struct regulator	*vref;
	struct gpio_chip	gpiochip;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*start_gpio;
	unsigned int		sampling_freq;
	unsigned int 		power_mode;
	unsigned int 		decimation;
	
	unsigned int 		crc_enabled;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_rx_buf[3] ____cacheline_aligned;
	u8			reg_tx_buf[3];
};

enum ad777x_data_lines {
	AD777x_4LINES,
	AD777x_2LINES,
	AD777x_1LINE,

};

static const char * const ad777x_filter_type_enum[] = {
	"SINC3",
	"SINC5"
};

static const char * const ad777x_data_lines_modes[] = {
	[AD777x_4LINES] = "4_data_lines",
	[AD777x_2LINES] = "2_data_lines",
	[AD777x_1LINE]  = "1_data_line",
};

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static bool ad7779_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad7779_state *ad7779_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if(ad7779_has_axi_adc(&indio_dev->dev)) {
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int ad7779_spi_read(struct iio_dev *indio_dev, u8 reg, u8 *rbuf) {
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
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
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
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

static int ad7779_reg_access(struct iio_dev *indio_dev,
				unsigned int reg,
				unsigned int writeval,
				unsigned int *readval) {
	int ret;

	if(readval)
		ret = ad7779_spi_read(indio_dev, reg, (u8 *) readval);
	else
		ret = ad7779_spi_write(indio_dev, reg, writeval);

	return ret;
}

static int ad7779_set_power_mode(struct iio_dev *indio_dev, 
				unsigned int power_mode)
{
	int ret;
	u8 temp;

	if(power_mode) {
		ret = ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, &temp);
		temp |= AD777X_POWER_MODE_HIGH;
		ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, temp);
	} else {
		ret = ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, &temp);
		temp |= AD777X_POWER_MODE_LOW;
		ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, temp);
	}

	return ret;
}

// static int ad777x_set_sampling_frequency(struct iio_dev *indio_dev,
// 					unsigned int sampling_freq)
// {
// 	int ret;
// 	unsigned int decimation;
// 	struct ad7779_state *st = ad7779_get_data(indio_dev);


// }

static int ad7779_spi_sar_mode_en(struct iio_dev *indio_dev)
{
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_2,
				AD7779_ENABLE_SAR);
	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_1,
				AD7779_PDB_SAR);

	return ret;
}

static int ad7779_spi_sar_mode_disable(struct iio_dev *indio_dev)
{
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

static int ad7779_data_read_disable(struct iio_dev *indio_dev) {
	int ret;

	ret = ad7779_spi_write(indio_dev,
				AD7779_REG_GENERAL_USER_CONFIG_3,
				AD7779_DISABLE_SD);

	return ret;
}

static int ad7779_sigma_delta_data(struct iio_dev *indio_dev) {
	int ret;
	struct ad7779_state *spi_st = ad7779_get_data(indio_dev);
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
	struct ad7779_state *st = ad7779_get_data(indio_dev);
	u8 temp;

	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ad7779_spi_read(indio_dev,
					AD7779_REG_CH_CONFIG(chan->channel),
					&temp);
		// ret = ad7779_spi_sar_diagnostic(indio_dev, &data);
		// ret = ad7779_sigma_delta_data(indio_dev);
		if(ret)
			return ret;
		*val = temp;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
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
	unsigned long dec;
	struct ad7779_state *st;

	switch(mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->sampling_freq = val;
		if(st->power_mode)
			dec = clk_get_rate(st->mclk) / (4 * val);
		else
			dec = clk_get_rate(st->mclk) / (8 * val);
		if(dec == ((int) dec)) {
			ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_MSB, ((int)dec & 0xFF));
			ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_LSB, (((int)dec << 8) & 0xFF));
		}
		else {
			return -EINVAL;
		}
		return 0;
	case IIO_CHAN_INFO_PHASE:
		return 0;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return 0;
	case IIO_CHAN_INFO_OFFSET:
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
	.debugfs_reg_access = &ad7779_reg_access,
};

#define AD777x_CHAN(index)							\
	{									\
		.type = IIO_VOLTAGE,						\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_OFFSET),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PHASE),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
		.address = index,						\
		.indexed = 1,							\
		.channel = index,						\
		.scan_index = index,						\
		.scan_type = {							\
			.sign = 's',						\
			.realbits = 24,						\
			.storagebits = 32,					\
		},								\
	}


static const struct iio_chan_spec ad7779_channels[] = {
	AD777x_CHAN(0),
	AD777x_CHAN(1),
	AD777x_CHAN(2),
	AD777x_CHAN(2),
	AD777x_CHAN(4),
	AD777x_CHAN(5),
	AD777x_CHAN(6),
	AD777x_CHAN(7),
};

static const struct axiadc_chip_info conv_chip_info = {
	.name = "ad7779_axi_adc",
	.max_rate = 2048000UL,
	.num_channels = 8,
	.channel[0] = AD777x_CHAN(0),
	.channel[1] = AD777x_CHAN(1),
	.channel[2] = AD777x_CHAN(2),
	.channel[3] = AD777x_CHAN(3),
	.channel[4] = AD777x_CHAN(4),
	.channel[5] = AD777x_CHAN(5),
	.channel[6] = AD777x_CHAN(6),
	.channel[7] = AD777x_CHAN(7),
};


static void ad7779_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void ad7779_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

int ad7779_gpio_init(struct ad7779_state *st) 
{
	st->gpiochip.label = "ad7779_gpios";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 2;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.owner = THIS_MODULE;

	return gpiochip_add_data(&st->gpiochip, st);
}

static int ad7779_register_axi(struct ad7779_state *st)
{
	struct axiadc_converter *conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->chip_info = &conv_chip_info;
	conv->reg_access = &ad7779_reg_access;
	conv->write_raw = &ad7779_write_raw;
	conv->read_raw = &ad7779_read_raw;
	conv->phy = st;
	spi_set_drvdata(st->spi, conv);
	dev_info(&st->spi->dev, "in the register axi function");

	return 0;
}

static int ad7779_register(struct ad7779_state *st, struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	dev_info(&st->spi->dev, "in the register function");

	//indio_dev->dev.parent = &st->spi->dev;
	indio_dev->name = "ad7779";
	indio_dev->info = &ad7779_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad7779_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7779_channels);
	dev_info(&st->spi->dev, "before buffer alloc");

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ad7779_spi_write(indio_dev,
			AD7779_REG_GENERAL_USER_CONFIG_3,
			0x80);

	ad7779_spi_write(indio_dev,
			AD7779_REG_GEN_ERR_REG_1_EN,
			AD7779_ENABLE_CRC);
	st->crc_enabled = true;

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad777x_powerup(struct iio_dev *indio_dev)
{
	int ret;
	struct ad7779_state *st = iio_priv(indio_dev);

	ret = ad7779_spi_write(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, 0x74);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_DOUT_FORMAT, 0x24);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_ADC_MUX_CONFIG, 0x40);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_N_LSB, 0x40);
	st->sampling_freq = 8000;

	if(ret)
		return ret;

	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x1);
	msleep(10);
	ret = ad7779_spi_write(indio_dev, AD7779_REG_SRC_UPDATE, 0x0);
	msleep(10);

	gpiod_set_value(st->start_gpio, 0);
	msleep(10);
	gpiod_set_value(st->start_gpio, 1);
	msleep(10);
	gpiod_set_value(st->start_gpio, 0);
	msleep(10);

	return ret;
}

static int ad7779_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;
	struct ad7779_state *ad7779_st;
	int ret;
	// u8 temp, temp2;
	// int rbuf;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad7779_st));
	if (!indio_dev)
		return -ENOMEM;
	
	ad7779_st = iio_priv(indio_dev);

	ad7779_st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(ad7779_st->vref))
		return PTR_ERR(ad7779_st->vref);

	ret = regulator_enable(ad7779_st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7779_reg_disable, ad7779_st->vref);
	if (ret)
		return ret;

	ad7779_st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(ad7779_st->mclk))
		return PTR_ERR(ad7779_st->mclk);

	ret = clk_prepare_enable(ad7779_st->mclk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad7779_clk_disable, ad7779_st->mclk);
	if (ret)
		return ret;

	ad7779_st->reset_gpio = devm_gpiod_get(&spi->dev, "resetn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad7779_st->reset_gpio))
		return PTR_ERR(ad7779_st->reset_gpio);

	ad7779_st->start_gpio = devm_gpiod_get(&spi->dev, "startn",
						GPIOD_OUT_HIGH);
	if (IS_ERR(ad7779_st->start_gpio))
		return PTR_ERR(ad7779_st->start_gpio);
	
	if (ad7779_st->reset_gpio) {
		gpiod_set_value(ad7779_st->reset_gpio, 1);
		usleep_range(225, 230);
		gpiod_set_value(ad7779_st->reset_gpio, 0);
		usleep_range(1, 2);
	}

	crc8_populate_msb(ad7779_crc8_table, AD7779_CRC8_POLY);
	ad7779_st->spi = spi;

	// ad7779_st->crc_enabled = false;

	/* write 0x3F to REG_GEN_ERR_1 to enable crc */
	//ad7779_st->crc_enabled = true;

	

	// ad7779_spi_sar_diagnostic(indio_dev, &rbuf);
	// dev_info(&spi->dev, "The data readback is %d", rbuf);
	//ad7779_soft_reset(indio_dev);

	// ad7779_spi_read(indio_dev, AD7779_REG_GEN_ERR_REG_1_EN, &temp);
	// dev_info(&spi->dev, "The value of general err reg en is %x", temp);

	// ad7779_spi_read(indio_dev, AD7779_REG_GEN_ERR_REG_1, &temp2);
	// dev_info(&spi->dev, "The value of general err reg 1 is %x", temp2);

	// ad7779_spi_read(indio_dev, AD7779_REG_GEN_ERR_REG_2, &temp2);
	// dev_info(&spi->dev, "The value of general err reg 2 is %x", temp2);


	// ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_1, &temp2);
	// dev_info(&spi->dev, "The value of general USER CONFIG is %x", temp2);


	// ad7779_spi_read(indio_dev, AD7779_REG_GENERAL_USER_CONFIG_2, &temp2);
	// dev_info(&spi->dev, "The value of general user config 2 is %x", temp2);


	// ad7779_spi_write(indio_dev,
	// 		AD7779_REG_GEN_ERR_REG_1_EN,
	// 		0x3E);
	// ad7779_st->crc_enabled = false;

	// ad7779_data_read_en(indio_dev);
	// dev_info(&spi->dev, "Enabled reading adc data via spi");
	// if (ad7779_st->reset_gpio) {
	// 	gpiod_direction_output(ad7779_st->reset_gpio, 0);
	// 	usleep_range(225, 230);
	// 	gpiod_direction_output(ad7779_st->reset_gpio, 1);
	// 	usleep_range(1, 2);
	// }

	ad7779_st->crc_enabled = false;
	ad777x_powerup(indio_dev);

	// ret = ad7779_register(ad7779_st, indio_dev);
	if (device_property_present(&spi->dev, "dmas"))
		ret = ad7779_register(ad7779_st, indio_dev);
	else
		ret = ad7779_register_axi(ad7779_st);

	return ret;
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
