// note: このコードはW25Qシリーズに合わせて最適化されています。他のFlashを使う際には見直す必要があります。
//       汎用性に気を使ったコードはオリジナル(fork元)を確認してください。
#include <stdio.h>

#include "flashrom.h"
#include "pico/bootrom.h"

#include "hardware/structs/ssi.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/flash.h"
#include "hardware/vreg.h"

#define FLASH_BLOCK_ERASE_CMD 0xd8

#if 1
#define BOOT2_SIZE_WORDS 64

static uint32_t boot2_copyout[BOOT2_SIZE_WORDS];
static bool boot2_copyout_valid = false;

static const struct FlashChip flash_chip_table[] = {
//     { 0xef, 0x4020, 4, 16, 190000, 0x4080, "W25Q512" },
    // { 0xef, 0x4020, 4, 16, (98000*3), 0x4063, 3, 2, "W25Q512" }, //ギリギリアウト
    { 0xef, 0x4020, 4, 16, (98000*3), 0x4a1A, 3, 2, "W25Q512" },
    // { 0xef, 0x4020, 4, 16, (295000), 0x4040, 3, 2, "W25Q512" },
    // { 0xef, 0x4020, 4, 16, (133000*2), 0x4040, 2, 2, "W25Q512" },
    { 0xef, 0x4019, 2, 16, 256000, 0x4022, 2, 1, "W25Q256" },
    { 0xef, 0x4018, 1, 16, 256000, 0x4022, 2, 1, "W25Q128" },
    { 0xef, 0x4017, 1, 8 , 256000, 0x4022, 2, 1, "W25Q64"  },
    { 0xef, 0x4016, 1, 4 , 256000, 0x4022, 2, 1, "W25Q32"  },
    { 0xef, 0x4015, 1, 2 , 256000, 0x4022, 2, 1, "W25Q16"  }
};

const struct FlashChip* flash_get_info() {
    
    uint8_t txbuf[4];
    uint8_t rxbuf[4];
    //    printf("Detect ROM chip\n");
    txbuf[0] = 0x9f;

    flash_do_cmd(txbuf, rxbuf, 4);

//    printf("Flash jedec id %02X %02X %02X\n", rxbuf[1], rxbuf[2], rxbuf[3]);

    uint8_t mf = rxbuf[1];
    uint16_t id = (rxbuf[2] << 8) | rxbuf[3];

    for (int i = 0; i < sizeof(flash_chip_table) / sizeof(struct FlashChip); i++) {
	    if (flash_chip_table[i].mf == mf && flash_chip_table[i].id == id) {
            return &flash_chip_table[i];
        }
    }
    return NULL;
}

void flash_set_config(const struct FlashChip* chip_info){
    vreg_set_voltage(VREG_VOLTAGE_1_10);
    ssi_hw->ssienr = 0;
    ssi_hw->baudr = chip_info->flash_clk_div;
    ssi_hw->rx_sample_dly = chip_info->flash_rx_delay;
    ssi_hw->ssienr = 1;
}

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

static void inline (flash_cs_force)(bool high) {
    uint32_t field_val = high ?
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_HIGH :
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW;
    hw_write_masked(&ioqspi_hw->io[1].ctrl,
        field_val << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
    );
}

static void xflash_do_cmd_internal(const uint8_t *txbuf, uint8_t *rxbuf, size_t count) 
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

#include "hardware/structs/xip_ctrl.h"
void inline static flash_trigger_flush_cache(){
    xip_ctrl_hw->flush = 1;
}
void inline static flash_wait_flush_cache(){
    // Read blocks until flush completion
    (void) xip_ctrl_hw->flush;
    // Enable the cache
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_EN_BITS);
}

void __no_inline_not_in_flash_func(flash_set_ea_reg)(uint8_t addr)
{
//      rom_connect_internal_flash_fn connect_internal_flash = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
//     rom_flash_exit_xip_fn flash_exit_xip = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
//     rom_flash_flush_cache_fn flash_flush_cache = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
//     assert(connect_internal_flash && flash_exit_xip && flash_flush_cache);
//     flash_init_boot2_copyout();
//     __compiler_memory_barrier();
//     connect_internal_flash();
//     flash_exit_xip();
//     uint8_t txbuf[4];
//     uint8_t rxbuf[4];

//     //https://github.com/raspberrypi/pico-bootrom/blob/ef22cd8ede5bc007f81d7f2416b48db90f313434/bootrom/program_flash_generic.c#L93

//    txbuf[0] = 0xc8;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("EA register %02X\n", rxbuf[1]);

//     txbuf[0] = 0x06;
//     xflash_do_cmd_internal(txbuf, rxbuf, 1);
//    txbuf[0] = 0x05;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("Status register %02X\n", rxbuf[1]);

//     txbuf[0] = 0xc5;
//     txbuf[1] = addr;
//     xflash_do_cmd_internal(txbuf, rxbuf, 2);

//     // lightのためにWrite Disableしない
//     // txbuf[0] = 0x04;
//     // xflash_do_cmd_internal(txbuf, rxbuf, 1);

//    txbuf[0] = 0xc8;
//    xflash_do_cmd_internal(txbuf, rxbuf, 2);
//    printf("EA register %02X\n", rxbuf[1]);
//     flash_flush_cache();
//     flash_enable_xip_via_boot2();
    flash_set_ea_reg_light(addr);
    // flash_trigger_flush_cache();
    // flash_wait_flush_cache();
}

// #define WAIT_SSI_TX() while( !(ssi_hw->sr & (SSI_SR_TFE_BITS)) )
// #define WAIT_SSI_TX() while( (ssi_hw->sr & (SSI_SR_BUSY_BITS)) )
#define WAIT_SSI_TX() while((ssi_hw->sr & (SSI_SR_TFE_BITS | SSI_SR_BUSY_BITS)) != SSI_SR_TFE_BITS)

void inline W25Q_enter_command_mode(){
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
    // (void) ssi_hw->sr;
    // (void) ssi_hw->icr;
    ssi_hw->ctrlr0 = qspi_txrx;   
    ssi_hw->spi_ctrlr0 = std_spi_ctrlr;    
    
    ssi_hw->ssienr = 1;
}

void inline xip_enter_command_mode(){

    W25Q_enter_command_mode();

    // Stop XIP cs control. by dummy command
    // const uint8_t txbuf = 0x9f;
    // uint8_t rxbuf;
    // xflash_do_cmd_internal(&txbuf, &rxbuf, 1);
    
    // flash_cs_force(0);
    ssi_hw->dr0 = 0x9f;
    WAIT_SSI_TX();
    // flash_cs_force(1);
}

void inline xip_exit_command_mode(){
    // https://github.com/raspberrypi/pico-sdk/blob/6a7db34ff63345a7badec79ebea3aaef1712f374/src/rp2_common/boot_stage2/boot2_w25q080.S
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
    // (void) ssi_hw->sr;
    // (void) ssi_hw->icr;
    ssi_hw->ctrlr0 = dummy_read_ctrl0;    
    ssi_hw->spi_ctrlr0 = dummy_read_spi_ctrl0;    

    // printf("iospi = %x\n",ioqspi_hw->io[1].ctrl & IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
    // hw_write_masked(&ioqspi_hw->io[1].ctrl,
    //     IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
    //     IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
    // );
    // printf("iospi = %x\n",ioqspi_hw->io[1].ctrl & IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS);
    ssi_hw->ssienr = 1;

    // flash_cs_force(0);
    ssi_hw->dr0 = 0xEB;
    ssi_hw->dr0 = 0xA0;

    // dummy readコマンド街
    WAIT_SSI_TX();


    // NEXT, XIP用にレジスタを構成

    const uint32_t xip_spi_ctrl0 =
    (0xa0 << SSI_SPI_CTRLR0_XIP_CMD_LSB) |     /* Mode bits to keep flash in continuous read mode */ 
    (8 << SSI_SPI_CTRLR0_ADDR_L_LSB) |         /* Total number of address + mode bits */ 
    (4 << SSI_SPI_CTRLR0_WAIT_CYCLES_LSB) |    /* Hi-Z dummy clocks following address + mode */ 
    (SSI_SPI_CTRLR0_INST_L_VALUE_NONE          /* Do not send a command, instead send XIP_CMD as mode bits after address */ 
        << SSI_SPI_CTRLR0_INST_L_LSB) | 
    (SSI_SPI_CTRLR0_TRANS_TYPE_VALUE_2C2A      /* Send Address in Quad I/O mode (and Command but that is zero bits long) */ 
        << SSI_SPI_CTRLR0_TRANS_TYPE_LSB);
      

    ssi_hw->ssienr = 0;
    // (void) ssi_hw->sr;
    // (void) ssi_hw->icr;
    ssi_hw->spi_ctrlr0 = xip_spi_ctrl0;    
      
    // hw_write_masked(&ioqspi_hw->io[1].ctrl,
    //     IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB,
    //     IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS
    // );

    ssi_hw->ssienr = 1;

    // for(int i=0;i<32;i++){
    //     printf("%02x ", ((uint8_t*)XIP_BASE)[i]);
    // }
    // printf(": XIP ACCESS after\n");
    
}

void flash_init_ea(){
    // xip_enter_command_mode();

    // // Disable write protect is executed in normal set_ea_reg. skip here.
    // const uint8_t txbuf[1] = {0x06};
    // uint8_t rxbuf[1];
    // xflash_do_cmd_internal(txbuf, rxbuf, 1);

    // xip_exit_command_mode();

}

#include <hardware/structs/systick.h>
// 時刻取得関数
inline uint32_t get_tick()
{
  return systick_hw->cvr;
}

// tick値の差分を計算
static uint32_t tick_diffs(uint32_t start_time, uint32_t end_time)
{
  if (start_time <= end_time) {
    // 測定時間内にreload発生
    start_time += systick_hw->rvr + 1;
  }
  return start_time - end_time;
}

void inline init_tick_timer(){
    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;
}


// #define LATCODE(x) x
#define LATCODE(x)

void flash_set_ea_reg_light(uint8_t addr)
{    
    // flash_trigger_flush_cache();

    LATCODE(uint32_t tick[10] = {0});
    LATCODE(size_t ti = 0);

    // uint8_t txbuf[2];
    // uint8_t rxbuf[2];

    LATCODE(tick[ti++] = get_tick());
    LATCODE(init_tick_timer());

    LATCODE(tick[ti++] = get_tick());
    xip_enter_command_mode();
   
    LATCODE(tick[ti++] = get_tick());
    // Disable write protect is executed in normal set_ea_reg. skip here.
    // txbuf[0] = 0x06;
    // xflash_do_cmd_internal(txbuf, rxbuf, 1);
    ssi_hw->dr0 = 0x06;
    WAIT_SSI_TX();
    LATCODE(tick[ti++] = get_tick());

    ssi_hw->dr0 = 0xc5;
    ssi_hw->dr0 = addr;
    WAIT_SSI_TX();


    // txbuf[0] = 0xc5;
    // txbuf[1] = addr;
    // xflash_do_cmd_internal(txbuf, rxbuf, 2);
    LATCODE(tick[ti++] = get_tick());

    // Enable write protect is not necesury.
    // txbuf[0] = 0x04;
    // xflash_do_cmd_internal(txbuf, rxbuf, 1);
   
    // flash_flush_cache();
    xip_exit_command_mode();
    LATCODE(tick[ti++] = get_tick());

    // flash_wait_flush_cache();
    LATCODE(tick[ti++] = get_tick());

    // printf("# flash EA REG %d\n", flash_get_ea_reg());
    
    LATCODE(printf("lat(%d) = init,%d ent,%d wp,%d ea,%d exi,%d flu,%d \n", ti, tick_diffs(tick[0], tick[1])
        , tick_diffs(tick[1], tick[2]), tick_diffs(tick[2], tick[3]), tick_diffs(tick[3], tick[4]), tick_diffs(tick[4], tick[5])
        , tick_diffs(tick[5], tick[6])));
        
    // printf("# flash EA REG %d\n", addr);
}

uint8_t __no_inline_not_in_flash_func(flash_get_ea_reg)(void)
{
#if 0
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
#else
    return 0xFF;
#endif
}
