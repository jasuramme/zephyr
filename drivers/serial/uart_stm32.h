/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on STM32 family processor.
 *
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_STM32_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_STM32_H_
#include <drivers/pinmux.h>
#include <drivers/uart.h>

/* device config */
struct uart_stm32_config {
	struct uart_device_config uconf;
	/* clock subsystem driving this peripheral */
	struct stm32_pclken pclken;
	/* initial hardware flow control, 1 for RTS/CTS */
	bool hw_flow_control;
	/* initial parity, 0 for none, 1 for odd, 2 for even */
	int  parity;
	const struct soc_gpio_pinctrl *pinctrl_list;
	size_t pinctrl_list_size;
};

/* driver data */
struct uart_stm32_data {
	/* Baud rate */
	uint32_t baud_rate;
	/* clock device */
	const struct device *clock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
        uart_callback_t async_callback;

        const uint8_t *volatile tx_buffer;
        size_t tx_buffer_length;
        size_t tx_counter;
        bool tx_abort_pending;
        struct k_timer *tx_timer;
        atomic_t tx_buffer_lock;

        uint8_t *volatile rx_buffer;
        uint8_t *volatile rx_secondary_buffer;
        size_t rx_buffer_length;
        size_t rx_secondary_buffer_length;
        size_t rx_counter;
        size_t rx_offset;
        int32_t rx_timeout;
        struct k_timer *rx_timer;
        atomic_t rx_buffer_lock;
        atomic_t rx_secondary_buffer_lock;
#endif
};

#endif	/* ZEPHYR_DRIVERS_SERIAL_UART_STM32_H_ */
