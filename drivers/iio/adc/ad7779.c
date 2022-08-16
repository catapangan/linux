/*
 * AD7779 ADC
 *
 * Copyright 2016-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

static const struct iio_info ad7779_info = {
};

static int ad7779_probe(struct spi_device *spi) {
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if(!indio_dev)
		return -ENOMEM;
	
	indio_dev->name = "ad7779";
	indio_dev->info = &ad7779_info;

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
