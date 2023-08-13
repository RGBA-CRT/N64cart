#include <stdio.h>

#include "flashrom.h"
#include "pico/bootrom.h"

#include "hardware/structs/ssi.h"
#include "hardware/structs/ioqspi.h"

#define FLASH_BLOCK_ERASE_CMD 0xd8

#if 1
#define BOOT2_SIZE_WORDS 64

static uint32_t boot2_copyout[BOOT2_SIZE_WORDS];
static bool boot2_copyout_valid = false;

static void __no_inline_not_in_flash_func(flash_init_boot2_copyout)(void) {
    if (boot2_copyout_valid)
        return;
    for (int i = 0; i < BOOT2_SIZE_WORDS; ++i)
        boot2_copyout[i] = ((uint32_t *)XIP_BASE)[i];
    __compiler_memory_barrier();
    boot2_copyout_valid = true;
}

static void __no_inline_not_in_flash_func(flash_enable_xip_via_boot2)(void) {
    ((void (*)(void))boot2_copyout+1)();
}
#else
static void __no_inline_not_in_flash_func(flash_init_boot2_copyout)(void) {}

static void __no_inline_not_in_flash_func(flash_enable_xip_via_boot2)(void) {
    // Set up XIP for 03h read on bus access (slow but generic)
    rom_flash_enter_cmd_xip_fn flash_enter_cmd_xip = (rom_flash_enter_cmd_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_ENTER_CMD_XIP);
    assert(flash_enter_cmd_xip);
    flash_enter_cmd_xip();
}
#endif

static void __no_inline_not_in_flash_func(flash_cs_force)(bool high) {
    uint32_t field_val = high ?
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH :
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
    hw_write_masked(&ioqspi_hw->io[1].ctrl,
        field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
    );
}

static void __no_inline_not_in_flash_func(xflash_do_cmd_internal)(const uint8_t *txbuf, uint8_t *rxbuf, size_t count) 
{
    flash_cs_force(0);
    size_t tx_remaining = count;
    size_t rx_remaining = count;
    // We may be interrupted -- don't want FIFO to overflow if we're distracted.
    const size_t max_in_flight = 16 - 2;
    while (tx_remaining || rx_remaining) {
        uint32_t flags = ssi_hw->sr;
        bool can_put = !!(flags & SSI_SR_TFNF_BITS);
        bool can_get = !!(flags & SSI_SR_RFNE_BITS);
        if (can_put && tx_remaining && rx_remaining - tx_remaining < max_in_flight) {
            ssi_hw->dr0 = *txbuf++;
            --tx_remaining;
        }
        if (can_get && rx_remaining) {
            *rxbuf++ = (uint8_t)ssi_hw->dr0;
            --rx_remaining;
        }
    }
    flash_cs_force(1);
}

void __no_inline_not_in_flash_func(flash_set_ea_reg)(uint8_t addr)
{
     rom_connect_internal_flash_fn connect_internal_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    assert(connect_internal_flash && flash_exit_xip && flash_flush_cache);
    flash_init_boot2_copyout();
    __compiler_memory_barrier();
    connect_internal_flash();
    flash_exit_xip();
    uint8_t txbuf[4];
    uint8_t rxbuf[4];

    //https://github.com/raspberrypi/pico-bootrom/blob/ef22cd8ede5bc007f81d7f2416b48db90f313434/bootrom/program_flash_generic.c#L93

   txbuf[0] = 0xc8;
   xflash_do_cmd_internal(txbuf, rxbuf, 2);
   printf("EA register %02X\n", rxbuf[1]);

    txbuf[0] = 0x06;
    xflash_do_cmd_internal(txbuf, rxbuf, 1);
   txbuf[0] = 0x05;
   xflash_do_cmd_internal(txbuf, rxbuf, 2);
   printf("Status register %02X\n", rxbuf[1]);

    txbuf[0] = 0xc5;
    txbuf[1] = addr;
    xflash_do_cmd_internal(txbuf, rxbuf, 2);

    txbuf[0] = 0x04;
    xflash_do_cmd_internal(txbuf, rxbuf, 1);

   txbuf[0] = 0xc8;
   xflash_do_cmd_internal(txbuf, rxbuf, 2);
   printf("EA register %02X\n", rxbuf[1]);
    flash_flush_cache();
    flash_enable_xip_via_boot2();
}

void flash_set_ea_reg_light(uint8_t addr)
{
    // flash_init_boot2_copyout();
    // __compiler_memory_barrier();
    // connect_internal_flash();
    // flash_exit_xip();
    uint8_t txbuf[8] = {0};
    uint8_t rxbuf[8]={0};

    uint32_t ctrl_back = ssi_hw->ctrlr0;
    uint32_t spi_ctrl_back = ssi_hw->spi_ctrlr0;
    const uint32_t qspi_txrx =
        (7 << SSI_CTRLR0_DFS_32_LSB) | /* 8 bits per data frame */ 
        (SSI_CTRLR0_TMOD_VALUE_TX_AND_RX << SSI_CTRLR0_TMOD_LSB) | /* Ena TXRX*/ 
        (SSI_CTRLR0_SPI_FRF_VALUE_STD    << SSI_CTRLR0_SPI_FRF_LSB);
    const uint32_t std_spi_ctrlr = 
        (0x03 << SSI_SPI_CTRLR0_XIP_CMD_LSB) |        /* Value of instruction prefix */ 
        (0 << SSI_SPI_CTRLR0_ADDR_L_LSB) |           /* Total number of address + mode bits */ 
        (2 << SSI_SPI_CTRLR0_INST_L_LSB) |                /* 8 bit command prefix (field value is bits divided by 4) */ 
        (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_1C1A << SSI_SPI_CTRLR0_TRANS_TYPE_LSB); /* command and address both in serial format */

    
    // (void) ssi_hw->sr;
    // (void) ssi_hw->icr;

    // コマンドはSPI、データは4ﾋﾞtで通信すり必要？
    //　QSPIではManufactireモード違い祖y、。

	// CLK Divider. rp2040@266MHz div=4, qspi_clk=66MHz
    // ssi_hw->baudr = 6;特別コメントアウト

    ssi_hw->ssienr = 0;
    ssi_hw->ctrlr0 = qspi_txrx;   
    ssi_hw->spi_ctrlr0 = std_spi_ctrlr;    
    
    ssi_hw->ser = 1;
    // Re-enable
    ssi_hw->ssienr = 1;

   txbuf[0] = 0x9f;
   xflash_do_cmd_internal(txbuf, rxbuf, 4);

    // for(int i=0;i<8;i++){
    //     printf("%02x ", ((uint8_t*)rxbuf)[i]);
    // }
    // printf(": QSPI READ DEVICE ID\n");
   
    // for(int i=0;i<0x100;i++){
    //     printf("%02x ", ((uint8_t*)XIP_SSI_BASE)[i]);
    // }
    // printf(": SSI REG DUMP\n");
    
   
   txbuf[0] = 0xc8;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("EA register %02X\n", rxbuf[1]);

    txbuf[0] = 0x06;
    xflash_do_cmd_internal(txbuf, rxbuf, 1);
//    txbuf[0] = 0x05;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("Status register %02X\n", rxbuf[1]);

    txbuf[0] = 0xc5;
    txbuf[1] = addr;
    xflash_do_cmd_internal(txbuf, rxbuf, 2);

    txbuf[0] = 0x04;
    xflash_do_cmd_internal(txbuf, rxbuf, 1);

//    txbuf[0] = 0xc8;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("EA register %02X\n", rxbuf[1]);

   
    ssi_hw->ssienr = 0;
    ssi_hw->ctrlr0 = ctrl_back;    
    ssi_hw->spi_ctrlr0 = spi_ctrl_back;    
    ssi_hw->ssienr = 1;

//     flash_flush_cache();
    // flash_enable_xip_via_boot2();
}

uint8_t __no_inline_not_in_flash_func(flash_get_ea_reg)(void)
{
    rom_connect_internal_flash_fn connect_internal_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    assert(connect_internal_flash && flash_exit_xip && flash_flush_cache);
    __compiler_memory_barrier();
    connect_internal_flash();
    flash_exit_xip();

    uint8_t txbuf[2];
    uint8_t rxbuf[2];

    txbuf[0] = 0xc8;
    xflash_do_cmd_internal(txbuf, rxbuf, 2);

    flash_flush_cache();
    flash_enable_xip_via_boot2();

    return rxbuf[1];
}
