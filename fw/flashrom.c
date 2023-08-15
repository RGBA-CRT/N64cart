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
    rom_flash_flush_cache_fn flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
    
    uint8_t txbuf[8] = {0};
    uint8_t rxbuf[8]={0};

    printf("iospi = %x\n",ioqspi_hw->io[1].ctrl & IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
    for(int i=0;i<0x100;i++){
        printf("%02x ", ((uint8_t*)XIP_BASE)[i]);
    }
    printf(": XIP ACCESS pre\n");

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

    ssi_hw->ssienr = 0;
    (void) ssi_hw->sr;
    (void) ssi_hw->icr;
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
    
   
//    txbuf[0] = 0xc8;
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
    // ssi_hw->ser = 0;

    // flash_enable_xip_via_boot2();

    /* flash_enable_xip_via_boot2 Details:
     * Check status register 2 to determine if QSPI mode is enabled, and perform an SR2 programming cycle if necessary.
     * ステータス レジスタ 2 をチェックして QSPI モードが有効かどうかを判断し、必要に応じて SR2 プログラミング サイクルを実行します。
     * 
     * Use SSI to perform a dummy 0xEB read command, with the modecontinuation bits set, so that the flash will not require0xEB instruction prefix on subsequent reads.
     * SSI を使用して、モード継続ビットを設定してダミー 0xEB 読み取りコマンドを実行します。これにより、フラッシュは後続の読み取りで 0xEB 命令プレフィックスを必要としません。
     * 
     * Configure SSI to write address, mode bits, but no instruction.
     * アドレス、モード ビットを書き込むように SSI を設定しますが、命令は書き込みません。
     * 
     * SSI + flash are now jointly in a state where continuous readscan take place.
     * SSI とフラッシュは共同で連続読み取りスキャンが行われる状態になります
     */
    const uint32_t dummy_read_ctrl0 =
        (SSI_CTRLR0_SPI_FRF_VALUE_QUAD << SSI_CTRLR0_SPI_FRF_LSB) |                                          
        (31 << SSI_CTRLR0_DFS_32_LSB)  |       /* 32 data bits (ADDR24 + Mode8) */                 
        (SSI_CTRLR0_TMOD_VALUE_EEPROM_READ << SSI_CTRLR0_TMOD_LSB);    /* Send INST/ADDR, Receive Data */ 

    const uint32_t dummy_read_spi_ctrl0 =
        ((32 /*bits*/ / 4) << SSI_SPI_CTRLR0_ADDR_L_LSB) |     /* Address + mode bits */
        (( 4 /* clocks */) << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) | /* Hi-Z dummy clocks following address + mode */
        (SSI_SPI_CTRLR0_INST_L_VALUE_8B << SSI_SPI_CTRLR0_INST_L_LSB) |        /* 8-bit instruction */
        (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_1C2A << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);      /* Send Command in serial mode then address in Quad I/O mode */
        
    ssi_hw->ssienr = 0;
    (void) ssi_hw->sr;
    (void) ssi_hw->icr;
    ssi_hw->ctrlr0 = dummy_read_ctrl0;    
    ssi_hw->spi_ctrlr0 = dummy_read_spi_ctrl0;    
    printf("iospi = %x\n",ioqspi_hw->io[1].ctrl & IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
 
    hw_write_masked(&ioqspi_hw->io[1].ctrl,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
    );
    printf("iospi = %x\n",ioqspi_hw->io[1].ctrl & IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
    ssi_hw->ssienr = 1;

    // flash_cs_force(0);
    ssi_hw->dr0 = 0xEB;
    ssi_hw->dr0 = 0xA0;

    // txbuf[0] = 0xc5;
    // txbuf[1] = addr;
    // xflash_do_cmd_internal(txbuf, rxbuf, 2);
    while((ssi_hw->sr & (SSI_SR_TFE_BITS | SSI_SR_BUSY_BITS)) == SSI_SR_TFE_BITS){

    printf("sr=%x\n", ssi_hw->sr);
    }
    printf("dr0=%x\n", ssi_hw->dr0);


    flash_flush_cache();

    const uint32_t xip_spi_ctrl0 =
    (0xa0                      /* Mode bits to keep flash in continuous read mode */ 
        << SSI_SPI_CTRLR0_XIP_CMD_LSB) | 
    (8 << SSI_SPI_CTRLR0_ADDR_L_LSB) |    /* Total number of address + mode bits */ 
    (4 << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |    /* Hi-Z dummy clocks following address + mode */ 
    (SSI_SPI_CTRLR0_INST_L_VALUE_NONE          /* Do not send a command, instead send XIP_CMD as mode bits after address */ 
        << SSI_SPI_CTRLR0_INST_L_LSB) | 
    (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_2C2A      /* Send Address in Quad I/O mode (and Command but that is zero bits long) */ 
        << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);

    ssi_hw->ssienr = 0;
    (void) ssi_hw->sr;
    (void) ssi_hw->icr;
    ssi_hw->spi_ctrlr0 = xip_spi_ctrl0;    
    ssi_hw->ssienr = 1;

    
    for(int i=0;i<0x100;i++){
        printf("%02x ", ((uint8_t*)XIP_BASE)[i]);
    }
    printf(": XIP ACCESS after\n");

    // printf("# flash EA REG %d\n", flash_get_ea_reg());
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
