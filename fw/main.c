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
#include "hardware/clocks.h"
#include "hardware/structs/ssi.h"
#include "hardware/flash.h"
#include "flashrom.h"

#include "main.h"
#include "cic.h"
#include "n64_pi.h"
#include "n64.h"

volatile uint32_t jpeg_start;

volatile uint32_t rom_pages;
volatile uint32_t rom_start[4];
volatile uint32_t rom_size[4];

static const char *rom_chip_name = NULL;

extern char __flash_binary_end;

static const struct FlashChip* g_flash_info = NULL;

static void setup_sysconfig(void)
{
    uintptr_t fw_binary_end = (uintptr_t) &__flash_binary_end;

    jpeg_start = ((fw_binary_end - XIP_BASE) + 4095) & ~4095;

    rom_pages = 1;
    rom_start[0] = ROM_BASE_RP2040 + jpeg_start + 64 * 1024;
    rom_size[0] = 2 * 1024 * 1024;

    g_flash_info = flash_get_info();
    if(g_flash_info){
	rom_pages = g_flash_info->rom_pages;
	rom_size[0] = g_flash_info->rom_size * 1024 * 1024;

	for (int p = 1; p < g_flash_info->rom_pages; p++) {
		rom_start[p] = ROM_BASE_RP2040 + 0;
		rom_size[p] = g_flash_info->rom_size * 1024 * 1024;
	}
	
	flash_set_config(g_flash_info);
	set_sys_clock_khz(g_flash_info->sys_freq, true);
	set_pi_bus_freq(g_flash_info->pi_bus_freq);
	rom_chip_name = g_flash_info->name;
     }

}

static void show_sysinfo(void)
{
    if (g_flash_info == NULL) {
	printf("Unknown ROM chip, system stopped!\n");
	while(1) {}
    }

    printf("ROM chip           : %s\n", g_flash_info->name);
    int clock_khz = clock_get_hz(clk_sys) / 1000;
    printf("System frequency   : %d.%03d MHz\n", clock_khz / 1000, clock_khz % 1000);
    printf("PI bus freq config : %04X\n\n", get_pi_bus_freq());
    printf("Available ROM pages:\n");

    for (int i = 0; i < rom_pages; i++) {
	printf("Page %d\n", i);
	printf(" Address %08X\n", rom_start[i]);
	printf(" Size    %d bytes\n", rom_size[i] - (rom_start[i] - ROM_BASE_RP2040));
    }
}

void n64_pi_restart(void)
{
    multicore_reset_core1();
    multicore_launch_core1(n64_pi);
}

#if PI_SRAM
const uint8_t __aligned(4096) __in_flash("n64_sram") n64_sram[SRAM_SIZE];

uint8_t sram_8[SRAM_SIZE];

bool g_is_n64_sram_write;

void n64_save_sram(void)
{
    uint32_t offset = ((uint32_t) n64_sram) - XIP_BASE;
    uint32_t count = sizeof(n64_sram);

    printf("backup SRAM to Flash...");

    flash_range_erase(offset, count);
    flash_range_program(offset, sram_8, count);
    
    flash_set_config(g_flash_info);
    
    printf("done.\n");
}
#endif

int main(void)
{
    for (int i = 0; i <= 27; i++) {
	gpio_init(i);
	gpio_set_dir(i, GPIO_IN);
	gpio_set_pulls(i, false, false);
    }

    gpio_init(N64_CIC_DCLK);
    gpio_init(N64_CIC_DIO);
    gpio_init(N64_COLD_RESET);
    gpio_init(N64_NMI);

    gpio_pull_up(N64_CIC_DIO);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    setup_sysconfig();

    stdio_init_all();
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

//     if(!g_flash_info){
// 	printf("unsupported flash type. abort.\n");
// 	while(1);
//     }
    show_sysinfo();

    flash_init_ea();
    flash_set_ea_reg(0);

    g_is_n64_sram_write = false;

    memcpy(sram_8, n64_sram, SRAM_SIZE);

    for(uint8_t j=0;j<rom_pages;j++){
       flash_set_ea_reg_light(j);
        for(int i=0;i<8;i++){
            printf("%02x ", ((uint8_t*)ROM_BASE_RP2040)[i+32]);
        }
	
        for(int i=0;i<8;i++){
            printf("%c", ((uint8_t*)ROM_BASE_RP2040)[i+32]);
        }
        printf(": Flash bank %d\n",j);
    }

    // flash_set_ea_reg(0x01);
    // printf("------- %d\n", flash_get_ea_reg());
    // flash_set_ea_reg_light(0x03);
    // printf("------- %d\n", flash_get_ea_reg());


    multicore_launch_core1(n64_pi);

    while (1) {
	/* main loop, N64 power on-offで1サイクル */

	/* reset to manu rom */
        flash_set_ea_reg(0);
	select_cic(CicType6102);

        cic_run();

	/* after n64 power off */
	if(g_is_n64_sram_write) {
        	flash_set_ea_reg(0);
		n64_save_sram();
		g_is_n64_sram_write = false;
	}
    }

    return 0;
}
