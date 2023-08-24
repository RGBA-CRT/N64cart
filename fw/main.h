#ifndef __MAIN_H__
#define __MAIN_H__

#define PI_SRAM 1

#define UART_TX_PIN (28)
#define UART_RX_PIN (29)	/* not available on the pico */
#define UART_ID     uart0
#define BAUD_RATE   921600

// #define SRAM_256KBIT_SIZE         0x00008000
// #define SRAM_768KBIT_SIZE         0x00018000
// #define SRAM_1MBIT_SIZE           0x00020000

/* SRAMは標準256KBitまで。対応すると処理時間が増えてしまう。
 * デザエモン3D以外はないはずなので不要に思う。
 * 将来的な拡張に備えてサイズは1Mbitで取っておく */
#define SRAM_SIZE 0x00020000

extern volatile uint32_t jpeg_start;

extern volatile uint8_t rom_pages;
extern volatile uint32_t rom_start[4];
extern volatile uint32_t rom_size[4];

void n64_pi_restart(void);

#if PI_SRAM
extern uint8_t sram_8[];
extern bool g_is_n64_sram_write;
void n64_save_sram(void);
#endif

/* キャッシュを使わない。flushのコストが高いため。 */
#define ROM_BASE_RP2040 XIP_NOCACHE_NOALLOC_BASE

#endif
