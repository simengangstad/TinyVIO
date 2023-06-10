#include "flexspi_boot.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)

/**
 * @brief Boot configuration.
 */
__attribute__((section(".boot_hdr.conf"), used))
const flexspi_nor_config_t qspiflash_config = {
    .memConfig =
        {
            .tag              = FLEXSPI_CFG_BLK_TAG,
            .version          = FLEXSPI_CFG_BLK_VERSION,
            .readSampleClkSrc = kFlexSPIReadSampleClk_LoopbackFromDqsPad,
            .csHoldTime       = 3u,
            .csSetupTime      = 3u,
            // Enable DDR mode, Wordaddassable, Safe configuration, Differential
            // clock
            .controllerMiscOption = 0x10,
            .deviceType           = kFlexSpiDeviceType_SerialNOR,
            .sflashPadType        = kSerialFlash_4Pads,
            .serialClkFreq        = kFlexSpiSerialClk_133MHz,
            .sflashA1Size         = 16u * 1024u * 1024u,
            .lookupTable =
                {
                    // Read LUTs
                    [0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                          FLEXSPI_1PAD,
                                          0xEB,
                                          RADDR_SDR,
                                          FLEXSPI_4PAD,
                                          0x18),
                    [1] = FLEXSPI_LUT_SEQ(DUMMY_SDR,
                                          FLEXSPI_4PAD,
                                          0x06,
                                          READ_SDR,
                                          FLEXSPI_4PAD,
                                          0x04),

                    // Read Status LUTs
                    [4 * 1 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x05,
                                                  READ_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x04),

                    // Write Enable LUTs
                    [4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x06,
                                                  STOP,
                                                  FLEXSPI_1PAD,
                                                  0x0),

                    // Erase Sector LUTs
                    [4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x20,
                                                  RADDR_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x18),

                    // Erase Block LUTs
                    [4 * 8 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                  FLEXSPI_1PAD,
                                                  0xD8,
                                                  RADDR_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x18),

                    // Pape Program LUTs
                    [4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x02,
                                                  RADDR_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x18),
                    [4 * 9 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR,
                                                  FLEXSPI_1PAD,
                                                  0x04,
                                                  STOP,
                                                  FLEXSPI_1PAD,
                                                  0x0),

                    // Erase Chip LUTs
                    [4 * 11 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR,
                                                   FLEXSPI_1PAD,
                                                   0x60,
                                                   STOP,
                                                   FLEXSPI_1PAD,
                                                   0x0),
                },
        },
    .pageSize           = 256u,
    .sectorSize         = 4u * 1024u,
    .ipcmdSerialClkFreq = 0x1,
    .blockSize          = 64u * 1024u,
    .isUniformBlockSize = false,
};

/**
 * @brief IVT Data.
 */
__attribute__((section(".boot_hdr.ivt"), used)) const ivt image_vector_table = {
    IVT_HEADER,                  // IVT Header
    IMAGE_ENTRY_ADDRESS,         // Image Entry Function
    IVT_RSVD,                    // Reserved = 0
    (uint32_t)DCD_ADDRESS,       // Address where DCD information is stored
    (uint32_t)BOOT_DATA_ADDRESS, // Address where BOOT Data Structure is stored
    (uint32_t)IVT_ADDRESS,       // Pointer to IVT Self (absolute address)
    (uint32_t)CSF_ADDRESS,       // Address where CSF file is stored
    IVT_RSVD                     // Reserved = 0
};

/**
 * @brief Boot data.
 */
__attribute__((section(".boot_hdr.boot_data"), used))
const BOOT_DATA_T g_boot_data = {
    BOOT_IMAGE_BASE, // boot start location
    BOOT_IMAGE_SIZE, // size
    PLUGIN_FLAG,     // Plugin flag
    0xFFFFFFFFU      // empty - extra data word
};

#endif

#if defined(__cplusplus)
}
#endif
