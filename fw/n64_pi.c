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

uint16_t *rom_file_16_piptr;
uint32_t flash_offset;
uint32_t n64_rom_size;
uint32_t inline address_n64_to_flash(uint32_t n64_rom_address){
	// n64_pi_address:  start ROM from 0x10000000 to 0x13FFFFFF
	// n64_rom_address: start ROM from 0x00000000 to +rom_size
	// flash_address:   firmware領域を飛ばす必要がある
	// TODO: あいまいさを排除, アドレスを定義する
	return (flash_offset + n64_rom_address) & 0x03FFFFFF;
}

// #define rom_file_16(adr) rom_file_16_piptr[adr]
#define rom_file_16(adr) flash_quad_read16_EC( address_n64_to_flash(adr) )

// static uint16_t *rom_jpeg_16;

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
	// __builtin_bswap32
	
//   asm ("rev16 %1,%0"
//           : "=r" (a)
//           : "r" (a));
//   return a;
    // 0x1122 => 0x2211
//     return (value << 8) | (value >> 8);
	return __builtin_bswap16(value);
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


#define FIFO_LOOK_AHEAD
uint32_t last_addr = 0;
uint8_t pi_last_page = 0;
uint32_t flash_bank_available;
// uint32_t pi_page_size;
uint8_t pi_game_page_origin;
uint32_t latency, max_byte_delay,max_byte_addr,latency_addr;
uint32_t bulk_start_copy, bulk_cnt;
#include "tick.h"


static uint32_t n64_bus_bulk_word_offset;
static inline void n64_bus_bulk_read_init(uint32_t address){
	// bulk_start = last_addr;
	if (address >= 0x10000000 && address <= 0x13FFFFFF) {
		n64_bus_bulk_word_offset = (address & 0x00FFFFFF)>>1;
	} else if (address >= 0x08000000 && address <= 0x0FFFFFFF) {
		n64_bus_bulk_word_offset = (address & (SRAM_SIZE-1))>>1;
		// bulk_start = last_addr;
	}
	
}
static inline uint16_t n64_bus_bulk_read_word(uint32_t n64_address){
	// バッファオーバーラン許して
	if (n64_address == 0x10000002) {
		n64_bus_bulk_word_offset++;
		return pi_bus_freq;
	} else if (n64_address >= 0x10000000 && n64_address <= 0x13FFFFFF) {
		return rom_file_16(n64_address & 0x03FFFFFF);
	} else if (n64_address >= 0x08000000 && n64_address <= 0x0FFFFFFF) {
		return sram_16[n64_bus_bulk_word_offset++];
		// return sram_16[ (address & (SRAM_SIZE-1))>>1 ];
	} else {
		return 0xDEAD;
	}
}

static void inline pi_bank_change(uint8_t page){
	rom_file_16_piptr = (uint16_t *) (rom_start[page]);
	flash_offset = page << 24;
	// flash_set_ea_reg_light(page);
	flash_set_ea_reg_light(0);
	// printf("flash_offset=%x\n", flash_offset);
}

void n64_pi(void)
{
//     rom_file_16 = (uint16_t *) ((uint32_t) rom_file | ROM_BASE_RP2040);
	// rom_file_16 = (uint16_t *) (ROM_BASE_RP2040 + rom_start[0]);
//     rom_jpeg_16 = (uint16_t *) (ROM_BASE_RP2040 + flash_sram_start);
	// n64_rom_size = 16*1024*1024

    PIO pio = pio0;
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &pi_program);
    pi_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    gpio_put(LED_PIN, 0);
    
//     tick_init();
//     uint32_t start;
    uint32_t bulk_start;

// n64_pi_start:
    printf("N64 PI engine start.\n");
    // Wait for reset to be released
    while (gpio_get(N64_COLD_RESET) == 0) {
	tight_loop_contents();
    }
//     bool romdisp_en = false;

last_addr=0x10000000;
    uint32_t word;
    uint32_t addr = pio_sm_get_blocking(pio, 0);
    do {    	
	if (addr == 0) {
	//     bulk_start = last_addr;
	//     n64_bus_bulk_read_init(last_addr);
	    pio_sm_clear_fifos(pio, 0);
	    // from PIO: READ REQUEST
		do {
		    word = n64_bus_bulk_read_word(last_addr);
		    last_addr+=2;
		    
		    pio_sm_put_blocking(pio, 0, swap8(word));
		//     printf("r%08X %04x\n", last_addr-2, word);
#ifdef FIFO_LOOK_AHEAD
		check_rx_fifo:
		    if(pio_sm_is_rx_fifo_empty(pio, 0)){
			// 指示はまだない
			if(pio_sm_is_tx_fifo_full(pio, 0)){
				// TXFIFOが詰まってやれることがない→指示待ち
				goto check_rx_fifo;
			}else{
				// TXIFOが空いているので突っ込んでしまえ
				continue;
			}
		    }
		    addr = pio_sm_get(pio, 0);
#else
		    addr = pio_sm_get_blocking(pio, 0);
#endif
		} while (addr == 0);
		
		// if((bulk_start>>24) == 0x08){
		// 	bulk_cnt = (last_addr-bulk_start);
		// 	bulk_start_copy = bulk_start;
		// }
		// if(romdisp_en)
		// 	printf("R%08X %d %x\n", bulk_start, bulk_cnt,addr);
			
		
	    	// last_addr=bulk_start;
		continue;
		
	    
	} else if (addr & 0x1) {
	//     bulk_start = last_addr;
		// printf("W%x %x\n", last_addr, addr);
	    // from PIO: WRITE
#if PI_SRAM
	    if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
		// bulk_start = last_addr;
		// if((last_addr == 0x08000002)){
		// 	last_addr-=2;
		// }
		// printf("W%x\n",bulk_start);
		// int write_margin = 0;
		n64_bus_bulk_word_offset = (last_addr & (SRAM_SIZE-1))>>1;
		bulk_start = n64_bus_bulk_word_offset;
		do {
		    sram_16[n64_bus_bulk_word_offset++] = (addr >> 16);
		//     sram_16[n64_bus_bulk_word_offset++] = swap8(addr >> 16);
		//     last_addr += 2;
		write_check_rx_fifo:
		    if(pio_sm_is_rx_fifo_empty(pio, 0)){
			// write_margin++;
			goto write_check_rx_fifo;
		    }
		    addr = pio_sm_get(pio, 0);
		} while (addr & 0x01);
		
		bulk_cnt=(n64_bus_bulk_word_offset-bulk_start)<<1 | 0x80000000;
		// bulk_cnt=write_margin<<16 | (last_addr-bulk_start) | 0x80000000;
		bulk_start_copy = last_addr;
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
	    }else{
		//  printf("WD_%x\n", last_addr);
		bulk_cnt= 0x80000000 | 4444;
		bulk_start_copy = last_addr;
	    }

	    last_addr += 2;

	} else {
#ifdef FIFO_LOOK_AHEAD
		//  pio_sm_clear_fifos(pio, 0);
#endif
		// start = tick_get();
	    /* from PIO: ADDRESS SET*/
	    last_addr = addr;
	//     if  (last_addr >= 0x10000000 && last_addr <= 0x1FBFFFFF) {
	// 	// non-guard outter rom access (map all pages)
	// 	// uint8_t page = (pi_game_page_origin + (uint8_t)(last_addr>>24) % rom_pages;
	// 	// guard outer rom access
	// 	uint8_t page = (pi_game_page_origin + (uint8_t)((last_addr>>24) % (n64_rom_size>>24)) ) % rom_pages;
	// 	if(page != pi_last_page) {
	// 		// if(last_addr >= 0x14000000){
	// 		// 	printf("ILL%x\n", last_addr);
	// 		// }
	// 		// if(n64_rom_offset >= n64_rom_size){
	// 		// 	// printf("OSA%x\n", last_addr);
	// 		// 	// n64_rom_offset %= n64_rom_size;
	// 		// }
	// 		// TODO: まともなマッピングシステムを考える。
	// 		// 今は１６MBページごと。page 0は16MBフルに使えないのでファーストバンクには選べない。
		
	// 		pi_bank_change(page);
			
	// 		// gpio_put(LED_PIN, page ? 1 :0);
	// 		pi_last_page = page;
	// 	}
	// 	// if( (last_addr & 0x0FFFFFFF) > n64_rom_size){
	// 	// 	printf("%x\n", last_addr);
	// 	// }
	//     } else if (last_addr >= 0x08000000 && last_addr <= 0x0FFFFFFF) {
	// 	// addr = pio_sm_get_blocking(pio, 0);
	// 	// continue;
	//     } else {
	// 	addr = pio_sm_get_blocking(pio, 0);
	// 	continue;
	//     }
	}
	if(pio_sm_is_rx_fifo_empty(pio, 0)){
		addr = 0;
	}else{
		addr = pio_sm_get(pio, 0);
	}
    } while (1);

//     goto n64_pi_start;
}


uint32_t inline rom_file_32(uint32_t addr){
	return rom_file_16(addr) | rom_file_16(addr+2)<<16;
}

uint8_t inline rom_file_8(uint32_t addr){
	// ROMには16bit LEで保存されているので、8bit単位(BEっぽく)アクセスするためにxorでbyte swapする
	return rom_file_16(addr ^ 1) >> 8;
}


void slove_mapped_rom(int id){
	n64_rom_size = rom_size[id];
	for(int i=id+1; i<(id+rom_pages-1); i++){
		int cur_id = i % rom_pages;
		pi_bank_change(cur_id);
		enum CicType type = cic_easy_detect(rom_file_32(CIC_DETECT_OFFSET));
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
void dump_n64(uint32_t address, size_t size){
    for(int of=0; of<size;of+=16){
        printf("%08x ",address+of);
        for(int i=0;i<16;i++){
            printf("%02x ",rom_file_8(address+of+i));
        }
        for(int i=0;i<16;i++){
            uint8_t b=rom_file_8(address+of+i);
            printf("%c",(b<0x20) ? '.' : ((b>=0x80) ? '.' : b));
        }
        printf("\n");
    }
}
void dump_n64_16(uint32_t address, size_t size){
    for(int of=0; of<size;of+=16){
        printf("%08x ",address+of);
        for(int i=0;i<16;i+=2){
            printf("%04x ",rom_file_16(address+of+i));
        }
        printf("\n");
    }
}
void dump_n64_32(uint32_t address, size_t size){
    for(int of=0; of<size;of+=16){
        printf("%08x ",address+of);
        for(int i=0;i<16;i+=4){
            printf("%08x ",rom_file_32(address+of+i));
        }
        printf("\n");
    }
}
void memcpy_n64(void* dst, uint32_t src, uint32_t size){
	for(uint32_t i=0;i<size/2; i++){
		((uint16_t*)dst)[i] = rom_file_16(src+(i*2));
	}
}

// #define TRACE(...) printf(__VA_ARGS__)
#define TRACE(...) 
void game_select(int id){
	id = id % rom_pages;
	slove_mapped_rom(id);

	pi_bank_change(id);
	// pi_page_size = rom_size[id];
	pi_game_page_origin = id;

	char title[22]={"DUMMY"};
	memcpy_n64(title, 0x00000020, 20);
	title[20] = '\0';

	if(strcmp("PicoCart64 Test ROM", title) == 0){
		printf("special debug flat mapping\n");
		n64_rom_size = 16*1024*1024*4;
	}

	enum CicType type = cic_easy_detect(rom_file_32(CIC_DETECT_OFFSET));
	select_cic(type);

	printf("Select bank:%d, CIC type: %s, title: %s, %dMB\n", id, cic_get_name(type), title, n64_rom_size/1024/1024);

	// debug rom access
	dump_n64(0, 16*4);
	// dump_n64_16(0, 16*4);
	// dump_n64_32(0, 16*4);

}