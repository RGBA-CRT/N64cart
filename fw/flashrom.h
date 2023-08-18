#ifndef __FLASHROM_H__
#define __FLASHROM_H__

#include <stdint.h>

struct FlashChip {
    uint8_t mf;
    uint16_t id;
    uint8_t rom_pages;
    uint8_t rom_size;
    uint32_t sys_freq;
    uint16_t pi_bus_freq;
    uint16_t flash_clk_div;
    uint8_t flash_rx_delay;
    const char *name;
};

void flash_init_ea();

void flash_set_ea_reg(uint8_t addr);
void flash_set_ea_reg_light(uint8_t addr);

uint8_t flash_get_ea_reg(void);

const struct FlashChip* flash_get_info();
void flash_set_config(const struct FlashChip* chip_info);

#endif
