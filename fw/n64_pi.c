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
uint32_t inline address_n64_rom_to_flash(uint32_t n64_rom_address){
	// n64_pi_address:  start ROM from 0x10000000 to 0x13FFFFFF
	// n64_rom_address: start ROM from 0x00000000 to +rom_size
	// flash_address:   firmware領域を飛ばす必要がある
	// TODO: あいまいさを排除, アドレスを定義する
	return (flash_offset + n64_rom_address) & 0x03FFFFFF;
}

// #define rom_file_16(adr) rom_file_16_piptr[adr]
#define rom_file_16(adr) flash_quad_read16_EC( address_n64_rom_to_flash(adr) )

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

PIO pio = pio0;

#define FIFO_LOOK_AHEAD
#define SRAM_LOOK_AHEAD
uint32_t last_addr = 0;
uint8_t pi_last_page = 0;
uint32_t flash_bank_available;
// uint32_t pi_page_size;
uint8_t pi_game_page_origin;
uint32_t latency, max_byte_delay,max_byte_addr,latency_addr;
uint32_t bulk_start_copy, bulk_cnt;
uint32_t debug_value = 0;
#include "tick.h"

uint32_t bulk_start;

static uint32_t n64_pi_bulk_offset;
static uint32_t n64_pi_bulk_write_offset;

typedef uint16_t(* N64_PI_READ_FUNC)();
typedef void(* N64_PI_WRITE_FUNC)(uint16_t);

N64_PI_READ_FUNC n64_pi_read_routine = NULL;
N64_PI_WRITE_FUNC n64_pi_write_routine = NULL;

uint16_t n64_pi_read_rom(){
	uint16_t ret;
	if(n64_pi_bulk_offset != 1){
		ret = rom_file_16(n64_pi_bulk_offset << 1);
	} else {
		ret = pi_bus_freq;
	}
	n64_pi_bulk_offset;
	return swap8(ret);
}

// SRAM area
uint16_t n64_pi_read_sram(){
	// if(n64_pi_bulk_offset==256){
	// 	printf("bake val: %04x\n", sram_16[n64_pi_bulk_offset]);
	// }
	return sram_16[n64_pi_bulk_offset];
}
void n64_pi_write_sram(uint16_t data){
	// if(g_is_n64_sram_write){
	// 	printf("w%04x\n",data);
	// }
	sram_16[n64_pi_bulk_write_offset] = data;
}

// invalid area
uint16_t n64_pi_read_invalid_area(){
	return 0xDEAD;
}
void n64_pi_write_invalid_area(uint16_t data){
	bulk_cnt= 0xC0000000 | data;
	bulk_start_copy = n64_pi_bulk_offset;
}

// LAT期間にできる処理
static inline void n64_pi_address_decode(uint32_t n64_pi_addr){
	// オカリナのSRAMアクセスが早すぎるので分岐を最優先で行う
	if (n64_pi_addr >= 0x08000000 && n64_pi_addr <= 0x0FFFFFFF) {
		/* SRAM */
		n64_pi_bulk_offset = (n64_pi_addr & (SRAM_SIZE-1))>>1;
#ifdef SRAM_LOOK_AHEAD
		pio_sm_put(pio, 0, 0xFFF00000 | (sram_16[n64_pi_bulk_offset]));
		pio_sm_put(pio, 0, 0xFFF10000 | (sram_16[n64_pi_bulk_offset+1]));
		// pio_sm_put(pio, 0, /*0xFFFF0000 |*/ (word));
		// pio_sm_put(pio, 0, /*0xFFFF0000 |*/ 0xCAFE);
		n64_pi_bulk_offset+=2;
#endif
		n64_pi_read_routine = n64_pi_read_sram;
		n64_pi_write_routine = n64_pi_write_sram;
		bulk_start = n64_pi_bulk_offset;
		n64_pi_bulk_write_offset = (n64_pi_addr & (SRAM_SIZE-1))>>1;
		// if(g_is_n64_sram_write){
		// 	printf("a%x\n",n64_pi_addr);
		// }
		// SRAM PRELOAD
		

	} else if (n64_pi_addr >= 0x10000000 && n64_pi_addr <= 0x13FFFFFF) {
		/* ROM */
		n64_pi_bulk_offset = (n64_pi_addr & 0x03FFFFFF)>>1;
		n64_pi_read_routine = n64_pi_read_rom;
		n64_pi_write_routine = n64_pi_write_invalid_area;

	} else {
		/* INVALID SPACE */
		n64_pi_bulk_offset = n64_pi_addr;
		n64_pi_read_routine = n64_pi_read_invalid_area;
		n64_pi_write_routine = n64_pi_write_invalid_area;
	}
}

static inline void n64_bus_bulk_read_exit(uint32_t n64_pi_addr){
	if (n64_pi_addr >= 0x08000000 && n64_pi_addr <= 0x0FFFFFFF) {
		bulk_cnt=(n64_pi_bulk_offset-bulk_start)<<1;
		bulk_start_copy = n64_pi_addr;
	}
}

static inline void n64_bus_bulk_write_exit(uint32_t n64_pi_addr){
	if (n64_pi_addr >= 0x08000000 && n64_pi_addr <= 0x0FFFFFFF) {
		bulk_cnt=(n64_pi_bulk_offset-bulk_start)<<1 | 0x80000000;
		bulk_start_copy = n64_pi_addr;
		g_is_n64_sram_write = true;
	}
}

static inline void n64_bus_bulk_write_word(uint32_t n64_pi_addr, uint16_t data){
	if (n64_pi_addr >= 0x08000000 && n64_pi_addr <= 0x0FFFFFFF) {
		    sram_16[n64_pi_bulk_offset++] = data;
	}
	else if (n64_pi_addr == 0x1fd01006) {
		uart_get_hw(UART_ID)->dr = data & 0xff;
	} else if (n64_pi_addr == 0x1fd0100a) {
		gpio_put(LED_PIN, data & 0x01);
	} else if (n64_pi_addr == 0x1fd0100e) {
		int page = data;
		game_select(page);
	}else{
		//  printf("WD_%x\n", n64_pi_addr);
		bulk_cnt= 0xC0000000 | data;
		bulk_start_copy = n64_pi_addr;
	}
}


static void inline pi_bank_change(uint8_t page){
	rom_file_16_piptr = (uint16_t *) (rom_start[page]);
	flash_offset = (page << 24) + ((uint32_t)rom_file_16_piptr-ROM_BASE_RP2040);
	// flash_set_ea_reg_light(page);
	flash_set_ea_reg_light(0);
	// printf("flash_offset=%x\n", flash_offset);
}

void n64_pi(void)
{
    // setup PIO
    pio_clear_instruction_memory(pio);
    uint offset = pio_add_program(pio, &pi_program);
    pi_program_init(pio, 0, offset);
    pio_sm_set_enabled(pio, 0, true);

    gpio_put(LED_PIN, 0);

    printf("N64 PI engine start.\n");

    // Wait for reset to be released
    while (gpio_get(N64_COLD_RESET) == 0) {
	tight_loop_contents();
    }

pio_sm_put(pio, 0, 0x00000000); // send "SYNC"
    last_addr=0x10000000;
    uint32_t word;
    uint32_t addr = pio_sm_get_blocking(pio, 0);
    do {
	// debug_value = addr;
	if (!addr) {
	    // from PIO: READ REQUEST
		while(1) {
		    word = n64_pi_read_routine();
		    pio_sm_put_blocking(pio, 0, 0xFFFF0000 | (word));
		    n64_pi_bulk_offset++;
		//      printf("r%08X %04x\n", last_addr, word);
#ifdef FIFO_LOOK_AHEAD
		check_rx_fifo:
		    if(pio_sm_is_rx_fifo_empty(pio, 0)){
			// 指示はまだない
			if(pio_sm_is_tx_fifo_full(pio, 0)){
				// TXFIFOが詰まってやれることがない→指示待ち
				//tight_loop_contents();
				goto check_rx_fifo;
			}else{
				// TXIFOが空いているので突っ込んでしまえ
				word = n64_pi_read_routine();
				continue;
			}
		    }
		    addr = pio_sm_get(pio, 0);
#else
		    addr = pio_sm_get_blocking(pio, 0);
#endif
		    // ahead2
		    if((addr)){break;}
		//     word = n64_pi_read_routine();
		};
	    pio_sm_put(pio, 0, 0x00000000); // send "SYNC"
	        // n64_bus_bulk_read_exit(last_addr);
		
		continue;	
	    
	} else if (addr & 0x1) {
	    // from PIO: WRITE	    
		do {
		    n64_pi_write_routine((addr >> 16));
		    n64_pi_bulk_write_offset++;
// write_check_rx_fifo:
// 		    if(pio_sm_is_rx_fifo_empty(pio, 0)){
// 			goto write_check_rx_fifo;
// 		    }
// 		    addr = pio_sm_get(pio, 0);
		    addr = pio_sm_get_blocking(pio, 0);
		} while (addr & 0x01);
	    pio_sm_put(pio, 0, 0x00000000); // send "SYNC"
				
	    n64_bus_bulk_write_exit(last_addr);

		continue;		

	} else {
	    /* from PIO: ADDRESS SET*/
	    n64_pi_address_decode(addr);
	    last_addr = addr; // TODO: 消す。完全に下の関数で管理するようにする

	    // preread
	//     word = n64_pi_read_routine();
	}

	addr = pio_sm_get_blocking(pio, 0);
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