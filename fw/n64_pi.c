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
#include "n64_pi.h"
#include "cic.h"

#include "rom.h"

static uint16_t *rom_file_16;
static uint16_t *rom_jpeg_16;

static uint16_t pi_bus_freq = 0x40ff;

#if PI_SRAM
static uint16_t *sram_16 = (uint16_t *) sram_8;

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

uint32_t last_addr = 0;
uint8_t pi_last_page = 0;
uint32_t flash_bank_available;
uint32_t n64_rom_size;
// uint32_t pi_page_size;
uint8_t pi_game_page_origin;


static void inline pi_bank_change(uint8_t page){
	rom_file_16 = (uint16_t *) (rom_start[page]);
	flash_set_ea_reg_light(page);
}

void n64_pi(void)
{
//     rom_file_16 = (uint16_t *) ((uint32_t) rom_file | ROM_BASE_RP2040);
	// rom_file_16 = (uint16_t *) (ROM_BASE_RP2040 + rom_start[0]);
    rom_jpeg_16 = (uint16_t *) (ROM_BASE_RP2040 + jpeg_start);
	// n64_rom_size = 16*1024*1024

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

    uint32_t word;

    uint32_t addr = pio_sm_get_blocking(pio, 0);
    do {
	if (addr == 0) {
	    // from PIO: READ REQUEST
	    if ((last_addr == 0x10000000)) {
		word = 0x3780;
		pio_sm_put(pio, 0, swap8(word));
		last_addr += 2;
#if PI_SRAM
		word = pi_bus_freq;
		//word = 0x40FF;
		//word = 0x4020;
#else
		word = pi_bus_freq;
		//word = 0x401c;
		//word = 0x4012;
#endif
		addr = pio_sm_get_blocking(pio, 0);
		if (addr == 0) {
		    goto hackentry;
		}

		continue;
	    } else if (last_addr >= 0x10000000 && last_addr <= 0x13FFFFFF/*0x1FBFFFFF*/) {
		do {
			// uint32_t n64_rom_offset = (last_addr & 0x03FFFFFF);
		//     uint32_t n64_16mb_bank = (last_addr & 0x03000000);
		    
		    uint32_t pi_xip_offset = (last_addr & 0x00FFFFFF);
		    word = rom_file_16[pi_xip_offset >> 1];
 hackentry:
		    pio_sm_put(pio, 0, swap8(word));
		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		    
		//     if(pi_xip_offset == 0x00FFFFFE){
		// 	printf("ov%x\n", last_addr>>24);
		//     }
		} while (addr == 0);

		continue;
#if PI_SRAM
	    } else if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
		do {
		    word = sram_16[(last_addr & (SRAM_SIZE-1)) >> 1];

		    pio_sm_put(pio, 0, word);
		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr == 0);

		continue;
#endif
	    } else if (last_addr >= 0x1fd80000) {
		do {
		    word = rom_jpeg_16[(last_addr & 0xFFFF) >> 1];

		    pio_sm_put(pio, 0, swap8(word));
		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr == 0);

		continue;
	    } else if (last_addr == 0x1fd01002) {
		word =
		    ((uart_get_hw(UART_ID)->fr & UART_UARTFR_TXFF_BITS) ? 0x00 : 0x0200) |
		    ((uart_get_hw(UART_ID)->fr & UART_UARTFR_RXFE_BITS) ? 0x00 : 0x0100) | 0xf000;
	    } else if (last_addr == 0x1fd01006) {
		word = uart_get_hw(UART_ID)->dr << 8;
	    } else if (last_addr == 0x1fd0100c) {
		word = rom_pages << 8;
	    } else if ((last_addr>>24) == 0x06) {
		/* 64DD not supported */
		word = 0x64dd;
	    } else {
		word = 0xdead;
		printf("D_%x\n", last_addr);
	    }
	    pio_sm_put(pio, 0, swap8(word));
	    last_addr += 2;
	} else if (addr & 0x1) {
	    // from PIO: WRITE
#if PI_SRAM
	    if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
		do {
		    sram_16[(last_addr & (SRAM_SIZE-1)) >> 1] = addr >> 16;

		    last_addr += 2;
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr & 0x01);
		g_is_n64_sram_write = true;

		continue;
	    } else 
#endif
	    if (last_addr == 0x1fd01006) {
		uart_get_hw(UART_ID)->dr = (addr >> 16) & 0xff;
	    } else if (last_addr == 0x1fd0100a) {
		gpio_put(LED_PIN, (addr >> 16) & 0x01);
	    } else if (last_addr == 0x1fd0100e) {
		int page = (addr >> 16);
		game_select(page);
	    }

	    last_addr += 2;

	} else {
	    /* from PIO: ADDRESS SET*/
	    last_addr = addr;
	    if  (last_addr >= 0x10000000 && last_addr <= 0x1FBFFFFF) {
		// non-guard outter rom access (map all pages)
		// uint8_t page = (pi_game_page_origin + (uint8_t)(last_addr>>24) % rom_pages;
		// guard outer rom access
		uint8_t page = (pi_game_page_origin + (uint8_t)((last_addr>>24) % (n64_rom_size>>24)) ) % rom_pages;
		if(page != pi_last_page) {
			// if(last_addr >= 0x14000000){
			// 	printf("ILL%x\n", last_addr);
			// }
			// if(n64_rom_offset >= n64_rom_size){
			// 	// printf("OSA%x\n", last_addr);
			// 	// n64_rom_offset %= n64_rom_size;
			// }
			// TODO: まともなマッピングシステムを考える。
			// 今は１６MBページごと。page 0は16MBフルに使えないのでファーストバンクには選べない。
		
			pi_bank_change(page);
			
			// gpio_put(LED_PIN, page ? 1 :0);
			pi_last_page = page;
		}
		if( (last_addr & 0x0FFFFFFF) > n64_rom_size){
			printf("%x\n", last_addr);
		}
	    }
	}
	addr = pio_sm_get_blocking(pio, 0);
    } while (1);
}


void slove_mapped_rom(int id){
	n64_rom_size = rom_size[id];
	for(int i=id+1; i<(id+rom_pages-1); i++){
		int cur_id = i % rom_pages;
		pi_bank_change(cur_id);
		enum CicType type = cic_easy_detect(*((uint32_t*)(&rom_file_16[CIC_DETECT_OFFSET/2])));
		if(type == _CicTypeMax_){
			n64_rom_size+=rom_size[cur_id];
		}else{
			break;
		}
		// printf("%s: page=%d %s size=%dMB \n", __func__, cur_id, cic_get_name(type), n64_rom_size/1024/1024);
	}
	// flash_set_ea_reg(id);
	// printf("%s: decision: %dMB\n", __func__, n64_rom_size/1024/1024);
}

void game_select(int id){
	id = id % rom_pages;
	slove_mapped_rom(id);

	pi_bank_change(id);
	// pi_page_size = rom_size[id];
	pi_game_page_origin = id;

	char title[21];
	strncpy(title, (char*)&rom_file_16[0x20/2], sizeof(title));
	title[20] = '\0';

	if(strcmp(title, "PicoCart64 Test ROM") == 0){
		printf("special debug flat mapping\n");
		n64_rom_size = 16*1024*1024*4;
	}

	enum CicType type = cic_easy_detect(*((uint32_t*)(&rom_file_16[CIC_DETECT_OFFSET/2])));
	select_cic(type);

	printf("Select bank:%d, CIC type: %s, title: %s, %dMB\n", id, cic_get_name(type), title, n64_rom_size/1024/1024);
}