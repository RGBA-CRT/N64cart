/**
 * Copyright (c) 2022 Konrad Beckmann
 *
 * Copyright (c) 2022 sashz /pdaXrom.org/
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "hardware/flash.h"
#include "flashrom.h"
#include "main.h"
#include "n64_pi.pio.h"
#include "n64.h"

static uint16_t *rom_base_16 = (uint16_t *) XIP_BASE;
uint16_t rom_lookup[16386];

static uint16_t pi_bus_freq = 0x40ff;

#if PI_SRAM
static uint16_t *sram_16 = (uint16_t *) sram_8;

static inline uint32_t resolve_sram_address(uint32_t address)
{
    uint32_t bank = (address >> 18) & 0x3;
    uint32_t resolved_address;

    if (bank) {
        resolved_address = address & (SRAM_256KBIT_SIZE - 1);
        resolved_address |= bank << 15;
    } else {
        resolved_address = address & (SRAM_1MBIT_SIZE - 1);
    }

    return resolved_address;
}
#endif

static inline uint32_t swap16(uint32_t value)
{
    // 0x11223344 => 0x33441122
    return (value << 16) | (value >> 16);
}

static inline uint32_t swap8(uint16_t value)
{
    // 0x1122 => 0x2211
    return (value << 8) | (value >> 8);
}

void set_pi_bus_freq(uint16_t freq)
{
    if (freq < 0x4012 && freq > 0x40FF) {
	freq = 0x40FF;
    }

    pi_bus_freq = freq;
}

uint16_t get_pi_bus_freq(void)
{
    return pi_bus_freq;
}

void n64_pi(void)
{
    PIO pio = pio0;
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &pi_program);
    pi_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    gpio_put(LED_PIN, 0);

    // Wait for reset to be released
    while (gpio_get(N64_COLD_RESET) == 0) {
	tight_loop_contents();
    }

    uint32_t last_addr = 0;
    uint32_t word;

    uint32_t mapped_addr;
    uint8_t ea;

    uint32_t addr = pio_sm_get_blocking(pio, 0);
    do {
	if (addr == 0) {
	    //READ
	    if (last_addr == 0x10000000) {
		word = 0x3780;
		pio_sm_put(pio, 0, swap8(word));
		last_addr += 2;
#if PI_SRAM
		word = pi_bus_freq;
//		word = 0x40FF;
#else
		word = pi_bus_freq;
//		word = 0x40FF;
#endif
		addr = pio_sm_get_blocking(pio, 0);
		if (addr == 0) {
		    goto hackentry;
		}

		continue;
	    } else if (last_addr >= 0x10000000 && last_addr <= 0x1FBFFFFF) {
		do {
		    mapped_addr = (rom_lookup[(last_addr & 0x3ffffff) >> 12]) << 12 | (last_addr & 0xfff);
		    ea = (mapped_addr >> 24) & 0x03; // max 64MB
		    word = rom_base_16[(mapped_addr & 0xffffff) >> 1];
 hackentry:
		    pio_sm_put(pio, 0, swap8(word));
		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr == 0);

		continue;
#if PI_SRAM
	    } else if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
		do {
		    word = sram_16[resolve_sram_address(last_addr) >> 1];

		    pio_sm_put(pio, 0, word);
		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr == 0);

		continue;
#endif
	    } else if (last_addr == 0x1fd01002) {
		word =
		    ((uart_get_hw(UART_ID)->fr & UART_UARTFR_TXFF_BITS) ? 0x00 : 0x0200) |
		    ((uart_get_hw(UART_ID)->fr & UART_UARTFR_RXFE_BITS) ? 0x00 : 0x0100) | 0xf000;
	    } else if (last_addr == 0x1fd01006) {
		word = uart_get_hw(UART_ID)->dr << 8;
	    } else if (last_addr == 0x1fd0100c) {
		word = rom_pages << 8;
	    } else {
		word = 0xdead;
	    }
	    pio_sm_put(pio, 0, swap8(word));
	    last_addr += 2;
	} else if (addr & 0x1) {
	    // WRITE
#if PI_SRAM
	    if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
		do {
		    sram_16[resolve_sram_address(last_addr) >> 1] = addr >> 16;

		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr & 0x01);

		continue;
	    } else 
#endif
	    if (last_addr == 0x1fd01006) {
		uart_get_hw(UART_ID)->dr = (addr >> 16) & 0xff;
	    } else if (last_addr == 0x1fd0100a) {
		gpio_put(LED_PIN, (addr >> 16) & 0x01);
	    }

	    last_addr += 2;
	} else {
	    last_addr = addr;
	}
	addr = pio_sm_get_blocking(pio, 0);
    } while (1);
}
