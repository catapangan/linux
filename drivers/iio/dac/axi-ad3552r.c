// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AXI AD3552R Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/dma-mapping.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/clk.h>

struct axi_ad3552r_priv {
	struct gpio_desc	*reset_gpio;
	void __iomem		*regs;
	struct clk		*ref_clk;
	bool			ddr;
	bool enable;
};

void axi_ad3552r_write(struct axi_ad3552r_priv *priv, u32 reg, u32 val)
{   pr_err("%s: 0x%x = 0x%x\n", __FUNCTION__, reg, val);
	iowrite32(val, priv->regs + reg);
}

u32 axi_ad3552r_read(struct axi_ad3552r_priv *priv, u32 reg)
{
	return ioread32(priv->regs + reg);
}

void axi_ad3552r_spi_write_8b(struct axi_ad3552r_priv *priv, u32 reg, u32 val, bool sdr_ddr_n)
{  u32 read_val;
	axi_ad3552r_write(priv, 0x84, val << 16);
	axi_ad3552r_write(priv, 0x8c, reg << 24);
	pr_err("%s: 0x%x\n", __FUNCTION__, axi_ad3552r_read(priv, 0x84));
	pr_err("%s: 0x%x\n", __FUNCTION__, axi_ad3552r_read(priv, 0x8c));
	read_val = axi_ad3552r_read(priv, 0x8c);
	pr_err("read value is : 0x%x\n", read_val);
	if (sdr_ddr_n) {
		axi_ad3552r_write(priv, 0x48, 0x14000);
		axi_ad3552r_write(priv, 0x8c, read_val | 0x00000001);
		mdelay(100);
		axi_ad3552r_write(priv, 0x8c, read_val & 0xffff0000);
	} else {
		axi_ad3552r_write(priv, 0x48, 0x04000);
		axi_ad3552r_write(priv, 0x8c, read_val | 0x00000001);
		mdelay(100);
		axi_ad3552r_write(priv, 0x8c, read_val & 0xffff0000);
	}
}

void axi_ad3552r_spi_write_16b(struct axi_ad3552r_priv *priv, u32 reg, u32 val, bool sdr_ddr_n)
{
	u32 read_val;
	axi_ad3552r_write(priv, 0x84, val << 8);
	axi_ad3552r_write(priv, 0x8c, reg << 24);
	pr_err("%s: 0x%x\n", __FUNCTION__, axi_ad3552r_read(priv, 0x84));
	pr_err("%s: 0x%x\n", __FUNCTION__, axi_ad3552r_read(priv, 0x8c));
	read_val = axi_ad3552r_read(priv, 0x8c);
	pr_err("read value is : 0x%x\n", read_val);
	if(sdr_ddr_n) {
		axi_ad3552r_write(priv, 0x48, 0x10000);
		axi_ad3552r_write(priv, 0x8c, read_val | 0x00000001);
		mdelay(100);
		axi_ad3552r_write(priv, 0x8c, read_val & 0xffff0000);
	} else {
		axi_ad3552r_write(priv, 0x48, 0x00000);
		axi_ad3552r_write(priv, 0x8c, read_val | 0x1);
		mdelay(100);
		axi_ad3552r_write(priv, 0x8c, read_val & 0xffff0000);
	}
}
static int axi_ad3552r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct axi_ad3552r_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (priv->ddr)
			*val = clk_get_rate(priv->ref_clk) / 8;
		else
			*val = clk_get_rate(priv->ref_clk) / 4;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		*val = priv->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->channel){
			axi_ad3552r_spi_write_16b(priv, 0xac, 0x00, 1);
			*val = axi_ad3552r_read(priv,0x80);
		} else {
			axi_ad3552r_spi_write_16b(priv, 0xaa, 0x00, 1);
			*val = axi_ad3552r_read(priv,0x80);
		}
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int axi_ad3552r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct axi_ad3552r_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return 0;
	case IIO_CHAN_INFO_RAW:
		if (chan->channel)
			axi_ad3552r_spi_write_16b(priv, 0x2c, (u32)val , 1);
		else
			axi_ad3552r_spi_write_16b(priv, 0x2a, (u32)val , 1);
	}

	return -EINVAL;
}

static int axi_ad3552r_reg_access(struct iio_dev *indio_dev,
				  unsigned reg, unsigned writeval,
				  unsigned *readval)
{
	struct axi_ad3552r_priv *priv = iio_priv(indio_dev);

	if (readval) {
		*readval = ioread32(priv->regs + reg);

		return 0;
	}

	iowrite32(writeval, priv->regs + reg);

	return 0;
}



static const struct iio_chan_spec axi_ad3552r_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.output = 1,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_BE,
		},
	}, {
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.output = 1,
		.indexed = 1,
		.channel = 1,
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_BE,
		},
	}
};


/*
 * ad7298_update_scan_mode() setup the spi transfer buffer for the new scan mask
 */
static int axi_ad3552r_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *active_scan_mask)
{  
	struct axi_ad3552r_priv *priv = iio_priv(indio_dev);
    pr_err("%s : %s \n", __FUNCTION__, indio_dev->name);
	if(!test_bit(0, active_scan_mask) && test_bit(1, active_scan_mask)){
	priv->ddr = true;
	// Stream length
	axi_ad3552r_spi_write_8b(priv, 0x0e, 0x02, 1);
	mdelay(100);
	// DDR configure
	axi_ad3552r_spi_write_8b(priv, 0x14, 0x05, 1);
	axi_ad3552r_write(priv, 0x8c, 0x2c000000);
    pr_err("Just channel 1 is active\n");

	} else if(test_bit(0, active_scan_mask) && !test_bit(1, active_scan_mask)) {
	// Stream length
	axi_ad3552r_spi_write_8b(priv, 0x0e, 0x02, 1);
	mdelay(100);
	// DDR configure
	axi_ad3552r_spi_write_8b(priv, 0x14, 0x05, 1);
	axi_ad3552r_write(priv, 0x8c, 0x2a000000);
    pr_err("Just channel 0 is active\n");
	} else {
	// Stream length
	axi_ad3552r_spi_write_8b(priv, 0x0e, 0x04, 1);
	mdelay(100);
	// DDR configure
	axi_ad3552r_spi_write_8b(priv, 0x14, 0x05, 1);

	axi_ad3552r_write(priv, 0x8c, 0x2c000000);
    pr_err("Both channels are active\n");
	}

	return 0;
}

static const struct iio_info axi_ad3552r_info = {
	.read_raw = &axi_ad3552r_read_raw,
	.write_raw = &axi_ad3552r_write_raw,
	.debugfs_reg_access = &axi_ad3552r_reg_access,
	.update_scan_mode = axi_ad3552r_update_scan_mode,
};


static int axi_ad3552r_buffer_postenable(struct iio_dev *indio_dev)
{   
	u32 read_val;
	struct axi_ad3552r_priv *priv = iio_priv(indio_dev);
	pr_err("%s : %s \n", __FUNCTION__, indio_dev->name);
	axi_ad3552r_write(priv, 0x48, 0x00010);
    // Start stream transfer ddr 
	read_val = axi_ad3552r_read(priv, 0x8c);
	pr_err("%s: 0x%x\n", __FUNCTION__, read_val);
	axi_ad3552r_write(priv, 0x8c, read_val | 0x00000003);
	
	return 0;
}

static int axi_ad3552r_buffer_postdisable(struct iio_dev *indio_dev)
{	
  struct axi_ad3552r_priv *priv = iio_priv(indio_dev);
    u32 read_val;	
	read_val = axi_ad3552r_read(priv, 0x8c);
	axi_ad3552r_write(priv, 0x8c, read_val && 0xffff0000);
	pr_err("%s: 0x%x\n", __FUNCTION__, axi_ad3552r_read(priv, 0x8c));
	axi_ad3552r_spi_write_8b(priv, 0x14, 0x04,0);
	priv->ddr = false;
	return 0;
}

static int axi_ad3552r_dma_buffer_submit(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_TO_DEVICE);
}

static const struct iio_buffer_setup_ops axi_ad3552r_buffer_setup_ops = {
	.postenable  = axi_ad3552r_buffer_postenable,
	.postdisable = axi_ad3552r_buffer_postdisable,
};

static const struct iio_dma_buffer_ops axi_ad3552r_dma_buffer_ops = {
	.submit = axi_ad3552r_dma_buffer_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static int axi_ad3552r_probe(struct platform_device *pdev)
{
	struct axi_ad3552r_priv *priv;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct resource *mem;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	priv->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->reset_gpio))
		return PTR_ERR(priv->reset_gpio);

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	mdelay(100);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	mdelay(100);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	priv->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->ref_clk))
		return PTR_ERR(priv->ref_clk);

	ret = clk_prepare_enable(priv->ref_clk);
	if (ret < 0)
		return ret;

	axi_ad3552r_write(priv, 0x40, 0x00);
	axi_ad3552r_write(priv, 0x40, 0x03);

	// External Vref + Idump
	axi_ad3552r_spi_write_8b(priv, 0x15, 0x00, 1);
	mdelay(100);
	// Stream mode enable
	axi_ad3552r_spi_write_8b(priv, 0x0f, 0x84, 1);
	mdelay(100);
	// Enable +-10V range
	axi_ad3552r_spi_write_8b(priv, 0x19, 0x44, 1);
	mdelay(100);
	axi_ad3552r_write(priv, 0x418, 0x02);
	axi_ad3552r_write(priv, 0x458, 0x02);
	priv->enable = false;

	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	indio_dev->setup_ops = &axi_ad3552r_buffer_setup_ops;
	indio_dev->channels = axi_ad3552r_channels;
	indio_dev->num_channels = ARRAY_SIZE(axi_ad3552r_channels);
	indio_dev->info = &axi_ad3552r_info;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
		&axi_ad3552r_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer)) {
		return PTR_ERR(buffer);
	}

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id axi_ad3552r_of_id[] = {
	{ .compatible = "adi,axi-ad3552r" },
	{ }
};
MODULE_DEVICE_TABLE(of, axi_ad3552r_of_id);

static struct platform_driver axi_ad3552r_driver = {
	.driver = {
		.name = "axi-ad3552r",
		.owner = THIS_MODULE,
		.of_match_table = axi_ad3552r_of_id,
	},
	.probe = axi_ad3552r_probe,
};
module_platform_driver(axi_ad3552r_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("AXI AD3552R Driver");
MODULE_LICENSE("GPL v2");
