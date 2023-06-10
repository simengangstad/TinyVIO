#include "clock_controller.h"

#include "fsl_device_registers.h"

#include "fsl_clock.h"
#include "fsl_common_arm.h"

#ifdef USE_LWIP
#include "arch/sys_arch.h"
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "fsl_iomuxc.h"
#include "fsl_pmu.h"
#pragma GCC diagnostic pop

static volatile uint32_t ms_since_boot = 0;

#ifdef __cplusplus
extern "C" {
#endif
#include "fsl_dcdc.h"

void SysTick_Handler(void) {

#ifdef USE_LWIP
    // Call the callback for LWIP, located in sys_arch.h to update its time
    time_isr();
#endif

    ms_since_boot++;
}

#ifdef __cplusplus
}
#endif

extern uint32_t SystemCoreClock;

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
#if defined(XIP_BOOT_HEADER_DCD_ENABLE) && (XIP_BOOT_HEADER_DCD_ENABLE == 1)

/**
 * @brief This function should not run from SDRAM as it will change the SEMC
 * configuration.
 */
AT_QUICKACCESS_SECTION_CODE(void update_semc_clock(void));

void update_semc_clock(void) {

    // Enable self-refresh mode
    //
    // The 16 first bits here are the key which allow to issue commands to the
    // SEMC
    SEMC->IPCMD = 0xA55A000D;
    while ((SEMC->INTR & 0x3) == 0) {}

    SEMC->INTR = 0x3;
    SEMC->DCCR = 0x0B;

    // Currently we are using SEMC parameter which fit both 166MHz and 200MHz,
    // only need to change the SEMC clock root here. If customer is using their
    // own DCD and want to switch from 166MHz to 200MHz, extra SEMC
    // configuration might need to be adjusted here to fine tune the SDRAM
    // performance
    CCM->CLOCK_ROOT[kCLOCK_Root_Semc].CONTROL = 0x602;
}
#endif
#endif

#ifndef SKIP_POWER_ADJUSTMENT
#if __CORTEX_M == 7
#define BYPASS_LDO_LPSR     1
#define SKIP_LDO_ADJUSTMENT 1
#elif __CORTEX_M == 4
#define SKIP_DCDC_ADJUSTMENT 1
#define SKIP_FBB_ENABLE      1
#endif
#endif

namespace clock_controller {
    /**
     * @brief The configuration for ARM PLL, which is clocked by an input
     * oscillator running at 24 MHz.
     *
     * The output frequency is given by:
     *
     * F_out = F_in * loopDivider / (2 * postDivider)
     *
     * Here we get the following frequency:
     *
     * F_out = 24 MHz * 166 / (2 * 2) = 996 MHz
     *
     * A higher loop divider will make us run out of the specification with the
     * given loop divider, so this is the closest we get to 1 GHz. The maximum
     * for the loop divider is also given by #ARM_PLL_DIV_SEL_MAX, which is 208,
     * so we can't use a higher post divider and try to get closer to 1 GHz with
     * the loop divider.
     */
    const clock_arm_pll_config_t arm_pll_config = {
        .postDivider = kCLOCK_PllPostDiv2,
        .loopDivider = 166,
    };

    /**
     * @brief Configuration for the system PLL 2.
     */
    const clock_sys_pll2_config_t sys_pll2_config = {
        /**
         * Denominator of spread spectrum
         */
        .mfd = 268435455,

        /**
         * Spread spectrum parameter
         */
        .ss = NULL,

        /**
         * Enable spread spectrum or not
         */
        .ssEnable = false,
    };

    /**
     * @brief Configuration for the video PLL.
     */
    const clock_video_pll_config_t video_pll_config = {
        /**
         * PLL Loop divider, valid range for DIV_SELECT divider value: 27 ~ 54.
         */
        .loopDivider = 41,

        /**
         * Divider after PLL, should only be 1, 2, 4, 8, 16, 32
         */
        .postDivider = 0,

        /**
         * 30 bit numerator of fractional loop divider:
         * F_out = F_in * (loopDivider + numerator / denominator)
         */
        .numerator   = 1,
        .denominator = 960000,

        /**
         * Spread spectrum parameter
         */
        .ss = NULL,

        /**
         * Enable spread spectrum or not
         */
        .ssEnable = false,
    };

    void initialise() {

#if __CORTEX_M == 7

        clock_root_config_t rootCfg = {};

        // Set DCDC to DCM mode to improve the efficiency for light loading in
        // run mode and transient performance with a big loading step
        DCDC_BootIntoDCM(DCDC);

#if !defined(SKIP_DCDC_ADJUSTMENT) || (!SKIP_DCDC_ADJUSTMENT)
        if ((OCOTP->FUSEN[16].FUSE == 0x57AC5969U) &&
            ((OCOTP->FUSEN[17].FUSE & 0xFFU) == 0x0BU)) {
            DCDC_SetVDD1P0BuckModeTargetVoltage(DCDC, kDCDC_1P0BuckTarget1P15V);
        } else {
            // Set 1.125V for production samples to align with data sheet
            // requirement
            DCDC_SetVDD1P0BuckModeTargetVoltage(DCDC,
                                                kDCDC_1P0BuckTarget1P125V);
        }
#endif

#if !defined(SKIP_FBB_ENABLE) || (!SKIP_FBB_ENABLE)
        // Check if FBB need to be enabled in OverDrive(OD) mode
        if (((OCOTP->FUSEN[7].FUSE & 0x10U) >> 4U) != 1) {
            PMU_EnableBodyBias(ANADIG_PMU, kPMU_FBB_CM7, true);
        } else {
            PMU_EnableBodyBias(ANADIG_PMU, kPMU_FBB_CM7, false);
        }
#endif

#if defined(BYPASS_LDO_LPSR) && BYPASS_LDO_LPSR
        PMU_StaticEnableLpsrAnaLdoBypassMode(ANADIG_LDO_SNVS, true);
        PMU_StaticEnableLpsrDigLdoBypassMode(ANADIG_LDO_SNVS, true);
#endif

#if !defined(SKIP_LDO_ADJUSTMENT) || (!SKIP_LDO_ADJUSTMENT)
        pmu_static_lpsr_ana_ldo_config_t lpsrAnaConfig;
        pmu_static_lpsr_dig_config_t lpsrDigConfig;

        if ((ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA &
             ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_MASK) == 0UL) {
            PMU_StaticGetLpsrAnaLdoDefaultConfig(&lpsrAnaConfig);
            PMU_StaticLpsrAnaLdoInit(ANADIG_LDO_SNVS, &lpsrAnaConfig);
        }

        if ((ANADIG_LDO_SNVS->PMU_LDO_LPSR_DIG &
             ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_BYPASS_MODE_MASK) == 0UL) {
            PMU_StaticGetLpsrDigLdoDefaultConfig(&lpsrDigConfig);
            lpsrDigConfig.targetVoltage = kPMU_LpsrDigTargetStableVoltage1P117V;
            PMU_StaticLpsrDigLdoInit(ANADIG_LDO_SNVS, &lpsrDigConfig);
        }
#endif

        // Config CLK_1M
        CLOCK_OSC_Set1MHzOutputBehavior(kCLOCK_1MHzOutEnableFreeRunning1Mhz);

        // Init OSC RC 16M
        ANADIG_OSC->OSC_16M_CTRL |= ANADIG_OSC_OSC_16M_CTRL_EN_IRC4M16M_MASK;

        // Init OSC RC 400M
        CLOCK_OSC_EnableOscRc400M();
        CLOCK_OSC_GateOscRc400M(true);

        // Init OSC RC 48M
        CLOCK_OSC_EnableOsc48M(true);
        CLOCK_OSC_EnableOsc48MDiv2(true);

        // Config OSC 24M
        ANADIG_OSC->OSC_24M_CTRL |= ANADIG_OSC_OSC_24M_CTRL_OSC_EN(1) |
                                    ANADIG_OSC_OSC_24M_CTRL_BYPASS_EN(0) |
                                    ANADIG_OSC_OSC_24M_CTRL_BYPASS_CLK(0) |
                                    ANADIG_OSC_OSC_24M_CTRL_LP_EN(1) |
                                    ANADIG_OSC_OSC_24M_CTRL_OSC_24M_GATE(0);

        // Wait for 24M OSC to be stable.
        while (ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE_MASK !=
               (ANADIG_OSC->OSC_24M_CTRL &
                ANADIG_OSC_OSC_24M_CTRL_OSC_24M_STABLE_MASK)) {}

        // Switch both cores, M7 Systick and Bus_Lpsr to OscRC48MDiv2 first
        rootCfg.mux = kCLOCK_M7_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);

        rootCfg.mux = kCLOCK_M7_SYSTICK_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M7_Systick, &rootCfg);

        rootCfg.mux = kCLOCK_M4_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);

        rootCfg.mux = kCLOCK_BUS_LPSR_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Bus_Lpsr, &rootCfg);

        // Init ARM PLL
        CLOCK_InitArmPll(&arm_pll_config);

        // Bypass SYS PLL1
        CLOCK_SetPllBypass(kCLOCK_PllSys1, true);

        // DeInit SYS PLL1
        CLOCK_DeinitSysPll1();

        // Init SYS PLL2
        CLOCK_InitSysPll2(&sys_pll2_config);
        CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd0, 27);
        CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd1, 16);
        CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd2, 24);
        CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, 32);

        // Init SYS PLL3
        CLOCK_InitSysPll3();
        CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd0, 13);
        CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd1, 17);
        CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd2, 32);
        CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd3, 22);

        // Bypass Audio PLL
        CLOCK_SetPllBypass(kCLOCK_PllAudio, true);
        CLOCK_DeinitAudioPll();

        // Init Video PLL
        // CLOCK_InitVideoPll(&video_pll_config);

        // -- Module clock root configurations --
        //
        // Now we configure the actual desired frequency for the cores and the
        // main bus

        // Configure M7 using ARM_PLL_CLK
        rootCfg.mux = kCLOCK_M7_ClockRoot_MuxArmPllOut;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);

        // Configure M4 using SYS_PLL3_PFD3_CLK
        rootCfg.mux = kCLOCK_M4_ClockRoot_MuxSysPll3Pfd3;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);

        // Configure BUS using SYS_PLL3_CLK. This is the main bus which e.g. is
        // clocked to AXI and thus OCRAM. PLL3 is running at 480 MHz, yielding
        // 240 MHz for the bus
        rootCfg.mux = kCLOCK_BUS_ClockRoot_MuxSysPll3Out;
        rootCfg.div = 2;
        CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootCfg);

        // Configure BUS_LPSR using SYS_PLL3_CLK
        rootCfg.mux = kCLOCK_BUS_LPSR_ClockRoot_MuxSysPll3Out;
        rootCfg.div = 3;
        CLOCK_SetRootClock(kCLOCK_Root_Bus_Lpsr, &rootCfg);

        // Configure SEMC using SYS_PLL2_PFD1_CLK
#ifndef SKIP_SEMC_INIT
        rootCfg.mux = kCLOCK_SEMC_ClockRoot_MuxSysPll2Pfd1;
        rootCfg.div = 3;
        CLOCK_SetRootClock(kCLOCK_Root_Semc, &rootCfg);
#endif

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
#if defined(XIP_BOOT_HEADER_DCD_ENABLE) && (XIP_BOOT_HEADER_DCD_ENABLE == 1)
        update_semc_clock();
#endif
#endif

        // Configure CSSYS using OSC_RC_48M_DIV2
        rootCfg.mux = kCLOCK_CSSYS_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Cssys, &rootCfg);

        // Configure CSTRACE using SYS_PLL2_CLK
        rootCfg.mux = kCLOCK_CSTRACE_ClockRoot_MuxSysPll2Out;
        rootCfg.div = 4;
        CLOCK_SetRootClock(kCLOCK_Root_Cstrace, &rootCfg);

        // Configure M4_SYSTICK using OSC_RC_48M_DIV2
        rootCfg.mux = kCLOCK_M4_SYSTICK_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_M4_Systick, &rootCfg);

        // Configure M7_SYSTICK using OSC_RC_48M_DIV2
        rootCfg.mux = kCLOCK_M7_SYSTICK_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 240;
        CLOCK_SetRootClock(kCLOCK_Root_M7_Systick, &rootCfg);

        // Configure FLEXIO1 using OSC_RC_48M_DIV2
        rootCfg.mux = kCLOCK_FLEXIO1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Flexio1, &rootCfg);

        // Configure FLEXIO2 using OSC_RC_48M_DIV2
        rootCfg.mux = kCLOCK_FLEXIO2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Flexio2, &rootCfg);

        /* Configure GPT1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt1, &rootCfg);

        /* Configure GPT2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt2, &rootCfg);

        /* Configure GPT3 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT3_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt3, &rootCfg);

        /* Configure GPT4 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT4_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt4, &rootCfg);

        /* Configure GPT5 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT5_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt5, &rootCfg);

        /* Configure GPT6 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_GPT6_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Gpt6, &rootCfg);

        /* Configure FLEXSPI1 using OSC_RC_48M_DIV2 */
#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1) || \
      defined(FLEXSPI_IN_USE))
        rootCfg.mux = kCLOCK_FLEXSPI1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Flexspi1, &rootCfg);
#endif

        /* Configure FLEXSPI2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_FLEXSPI2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Flexspi2, &rootCfg);

        /* Configure LPUART1 using SYS_PLL2_CLK */
        rootCfg.mux = kCLOCK_LPUART1_ClockRoot_MuxSysPll2Out;
        rootCfg.div = 22;
        CLOCK_SetRootClock(kCLOCK_Root_Lpuart1, &rootCfg);

        /* Configure EMV1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_EMV1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Emv1, &rootCfg);

        /* Configure EMV2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_EMV2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Emv2, &rootCfg);

        /* Configure ENET1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ENET1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Enet1, &rootCfg);

        /* Configure ENET2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ENET2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Enet2, &rootCfg);

        /* Configure ENET_25M using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ENET_25M_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Enet_25m, &rootCfg);

        /* Configure ENET_TIMER1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ENET_TIMER1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Enet_Timer1, &rootCfg);

        /* Configure ENET_TIMER2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ENET_TIMER2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Enet_Timer2, &rootCfg);

        /* Configure ASRC using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_ASRC_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Asrc, &rootCfg);

        /* Configure MQS using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_MQS_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Mqs, &rootCfg);

        /* Configure MIC using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_MIC_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Mic, &rootCfg);

        /* Configure SPDIF using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_SPDIF_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Spdif, &rootCfg);

        /* Configure SAI1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_SAI1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Sai1, &rootCfg);

        /* Configure SAI2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_SAI2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Sai2, &rootCfg);

        /* Configure SAI3 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_SAI3_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Sai3, &rootCfg);

        /* Configure SAI4 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_SAI4_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Sai4, &rootCfg);

        /* Configure CKO1 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_CKO1_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Cko1, &rootCfg);

        /* Configure CKO2 using OSC_RC_48M_DIV2 */
        rootCfg.mux = kCLOCK_CKO2_ClockRoot_MuxOscRc48MDiv2;
        rootCfg.div = 1;
        CLOCK_SetRootClock(kCLOCK_Root_Cko2, &rootCfg);

        /* Set SAI1 MCLK1 clock source. */
        IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk1Sel, 0);
        /* Set SAI1 MCLK2 clock source. */
        IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk2Sel, 3);
        /* Set SAI1 MCLK3 clock source. */
        IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI1MClk3Sel, 0);
        /* Set SAI2 MCLK3 clock source. */
        IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI2MClk3Sel, 0);
        /* Set SAI3 MCLK3 clock source. */
        IOMUXC_SetSaiMClkClockSource(IOMUXC_GPR, kIOMUXC_GPR_SAI3MClk3Sel, 0);

        /* Set MQS configuration. */
        IOMUXC_MQSConfig(IOMUXC_GPR, kIOMUXC_MqsPwmOverSampleRate32, 0);
        /* Set ENET Ref clock source. */
        IOMUXC_GPR->GPR4 &= ~IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK;
        /* Set ENET_1G Tx clock source. */
        IOMUXC_GPR->GPR5 = ((IOMUXC_GPR->GPR5 &
                             ~IOMUXC_GPR_GPR5_ENET1G_TX_CLK_SEL_MASK) |
                            IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK);
        /* Set ENET_1G Ref clock source. */
        IOMUXC_GPR->GPR5 &= ~IOMUXC_GPR_GPR5_ENET1G_REF_CLK_DIR_MASK;
        /* Set ENET_QOS Tx clock source. */
        IOMUXC_GPR->GPR6 &= ~IOMUXC_GPR_GPR6_ENET_QOS_RGMII_EN_MASK;
        /* Set ENET_QOS Ref clock source. */
        IOMUXC_GPR->GPR6 &= ~IOMUXC_GPR_GPR6_ENET_QOS_REF_CLK_DIR_MASK;
        /* Set GPT1 High frequency reference clock source. */
        IOMUXC_GPR->GPR22 &= ~IOMUXC_GPR_GPR22_REF_1M_CLK_GPT1_MASK;
        /* Set GPT2 High frequency reference clock source. */
        IOMUXC_GPR->GPR23 &= ~IOMUXC_GPR_GPR23_REF_1M_CLK_GPT2_MASK;
        /* Set GPT3 High frequency reference clock source. */
        IOMUXC_GPR->GPR24 &= ~IOMUXC_GPR_GPR24_REF_1M_CLK_GPT3_MASK;
        /* Set GPT4 High frequency reference clock source. */
        IOMUXC_GPR->GPR25 &= ~IOMUXC_GPR_GPR25_REF_1M_CLK_GPT4_MASK;
        /* Set GPT5 High frequency reference clock source. */
        IOMUXC_GPR->GPR26 &= ~IOMUXC_GPR_GPR26_REF_1M_CLK_GPT5_MASK;
        /* Set GPT6 High frequency reference clock source. */
        IOMUXC_GPR->GPR27 &= ~IOMUXC_GPR_GPR27_REF_1M_CLK_GPT6_MASK;

        SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);

        // Configure SysTick to count every millisecond
        SysTick_Config(USEC_TO_COUNT(1000U, SystemCoreClock));

#elif __CORTEX_M == 4
        // M7 is the main core, so we only configure the specific aspects for
        // the M4 core here as it is booted after the M7 core.

        SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M4);

        // Configure SysTick to count every millisecond
        SysTick_Config(USEC_TO_COUNT(1000U, SystemCoreClock));
#endif
    }

    uint32_t ms() { return ms_since_boot; }
}
