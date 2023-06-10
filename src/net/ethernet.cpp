#include "ethernet.h"

#include "lwip/init.h"
#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "fsl_enet.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#pragma GCC diagnostic pop

#include "ethernetif.h"
#include "fsl_phyksz8081.h"
#include "fsl_phyrtl8211f.h"
#include "fsl_silicon_id.h"
#include "netif/ethernet.h"

#include "board.h"
#include "logger.h"

/**
 * @brief Current ethernet port utilised.
 */
static ethernet::Port ethernet_port;

extern "C" {
void BOARD_ENETFlexibleConfigure(enet_config_t* config) {

    switch (ethernet_port) {
    case ethernet::Port::PORT_100M:
        config->miiMode = kENET_RmiiMode;
        break;
    case ethernet::Port::PORT_1G:
        config->miiMode = kENET_RgmiiMode;
        break;
    }
}
}

static phy_ksz8081_resource_t phy_ksz8081_resource;
static phy_rtl8211f_resource_t phy_rtl8211_resource;

static status_t mdio_write_ksz8081(uint8_t phy_address,
                                   uint8_t register_address,
                                   uint16_t data) {
    return ENET_MDIOWrite(ENET, phy_address, register_address, data);
}

static status_t mdio_read_ksz8081(uint8_t phy_address,
                                  uint8_t register_address,
                                  uint16_t* data_ptr) {
    return ENET_MDIORead(ENET, phy_address, register_address, data_ptr);
}

static status_t mdio_write_rtl8211(uint8_t phy_address,
                                   uint8_t register_address,
                                   uint16_t data) {
    return ENET_MDIOWrite(ENET_1G, phy_address, register_address, data);
}

static status_t mdio_read_rtl8211(uint8_t phy_address,
                                  uint8_t register_address,
                                  uint16_t* data_ptr) {
    return ENET_MDIORead(ENET_1G, phy_address, register_address, data_ptr);
}

namespace ethernet {

    /**
     * @brief Ethernet PHY handle.
     */
    static phy_handle_t phy_handle;

    /**
     * @brief Net interface handle.
     */
    static struct netif netif;

    /**
     * @brief Set to true once the module has been initialised.
     */
    static bool initialised = false;

    /**
     * @brief IP addresses for the MCU, the netmask and the gateway.
     */
    static ip4_addr_t netif_address, netif_net_mask, netif_gateway_address;

    bool initialise(const Port port,
                    const ip::IP4Address address,
                    const ip::IP4Address net_mask,
                    const ip::IP4Address gateway_address) {

        ethernet_port = port;

        // --- Clock configuration ---

        clock_sys_pll1_config_t sys_pll1_config = {.pllDiv2En = true,
                                                   .pllDiv5En = false,
                                                   .ss        = NULL,
                                                   .ssEnable  = false};

        CLOCK_InitSysPll1(&sys_pll1_config);

        CLOCK_EnableClock(kCLOCK_Iomuxc);

        switch (ethernet_port) {
        case Port::PORT_100M: {

            // 50MHz root clock for 100M ethernet.
            const clock_root_config_t root_config = {.clockOff = false,
                                                     .mux      = 4,
                                                     .div      = 10};

            CLOCK_SetRootClock(kCLOCK_Root_Enet1, &root_config);
            IOMUXC_GPR->GPR4 |= IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK;
            break;
        }

        case Port::PORT_1G: {

            // 125MHz root clock for 1G ethernet.
            const clock_root_config_t root_config = {.clockOff = false,
                                                     .mux      = 4,
                                                     .div      = 4};

            CLOCK_SetRootClock(kCLOCK_Root_Enet2, &root_config);
            IOMUXC_GPR->GPR5 |= IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK;

            break;
        }
        }

        // --- Pin configuration ---

        const gpio_pin_config_t ethernet_tranceiver_reset_pin_config = {
            .direction     = kGPIO_DigitalOutput,
            .outputLogic   = 0,
            .interruptMode = kGPIO_NoIntmode};

        switch (ethernet_port) {
        case Port::PORT_100M: {

            CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);

            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_12_GPIO9_IO11, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_32_ENET_MDC, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_AD_33_ENET_MDIO, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_02_ENET_TX_DATA00, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_03_ENET_TX_DATA01, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_04_ENET_TX_EN, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_05_ENET_REF_CLK, 1U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_06_ENET_RX_DATA00, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_07_ENET_RX_DATA01, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_08_ENET_RX_EN, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_09_ENET_RX_ER, 0U);
            IOMUXC_GPR->GPR4 = ((IOMUXC_GPR->GPR4 &
                                 (~(IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR_MASK))) |
                                IOMUXC_GPR_GPR4_ENET_REF_CLK_DIR(0x01U));
            IOMUXC_SetPinMux(IOMUXC_GPIO_LPSR_12_GPIO12_IO12, 0U);
            IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B2_05_ENET_REF_CLK, 0x03U);

            GPIO_PinInit(GPIO9, 11, &ethernet_tranceiver_reset_pin_config);
            GPIO_PinInit(GPIO12, 12, &ethernet_tranceiver_reset_pin_config);

            // Pull up the ENET_INT before RESET.
            GPIO_WritePinOutput(GPIO9, 11, 1);
            GPIO_WritePinOutput(GPIO12, 12, 0);
            SDK_DelayAtLeastUs(10000, CLOCK_GetFreq(kCLOCK_CpuClk));
            GPIO_WritePinOutput(GPIO12, 12, 1);
            SDK_DelayAtLeastUs(6, CLOCK_GetFreq(kCLOCK_CpuClk));

            break;
        }

        case Port::PORT_1G: {
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_00_ENET_1G_RX_EN, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_01_ENET_1G_RX_CLK, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_02_ENET_1G_RX_DATA00, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_03_ENET_1G_RX_DATA01, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_04_ENET_1G_RX_DATA02, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_05_ENET_1G_RX_DATA03, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_06_ENET_1G_TX_DATA03, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_07_ENET_1G_TX_DATA02, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_08_ENET_1G_TX_DATA01, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_09_ENET_1G_TX_DATA00, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_10_ENET_1G_TX_EN, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_11_ENET_1G_TX_CLK_IO, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B2_13_GPIO11_IO14, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_B2_19_ENET_1G_MDC, 0U);
            IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_B2_20_ENET_1G_MDIO, 0U);

            GPIO_PinInit(GPIO11, 14, &ethernet_tranceiver_reset_pin_config);

            // For a complete PHY reset of RTL8211FDI-CG, this pin must be
            // asserted low for at least 10ms. And wait for a further 30ms (for
            // internal circuits settling time) before accessing the PHY
            // register
            GPIO_WritePinOutput(GPIO11, 14, 0);
            SDK_DelayAtLeastUs(10000, CLOCK_GetFreq(kCLOCK_CpuClk));
            GPIO_WritePinOutput(GPIO11, 14, 1);
            SDK_DelayAtLeastUs(30000, CLOCK_GetFreq(kCLOCK_CpuClk));

            EnableIRQ(ENET_1G_MAC0_Tx_Rx_1_IRQn);
            EnableIRQ(ENET_1G_MAC0_Tx_Rx_2_IRQn);

            break;
        }
        }

        // --- Ethernet config ---

        ethernetif_config_t ethernet_config;
        ethernet_config.phyHandle  = &phy_handle;
        ethernet_config.srcClockHz = CLOCK_GetRootClockFreq(kCLOCK_Root_Bus);

        switch (ethernet_port) {

        case Port::PORT_100M:
            ethernet_config.phyAddr     = BOARD_ENET0_PHY_ADDRESS;
            ethernet_config.phyOps      = &phyksz8081_ops;
            ethernet_config.phyResource = &phy_ksz8081_resource;
            break;

        case Port::PORT_1G:
            ethernet_config.phyAddr     = BOARD_ENET1_PHY_ADDRESS;
            ethernet_config.phyOps      = &phyrtl8211f_ops;
            ethernet_config.phyResource = &phy_rtl8211_resource;

            break;
        }

        // --- MDIO ---
        switch (ethernet_port) {
        case Port::PORT_100M: {
            CLOCK_EnableClock(s_enetClock[ENET_GetInstance(ENET)]);
            ENET_SetSMI(ENET, CLOCK_GetRootClockFreq(kCLOCK_Root_Bus), false);

            phy_ksz8081_resource.read  = mdio_read_ksz8081;
            phy_ksz8081_resource.write = mdio_write_ksz8081;

            break;
        }
        case Port::PORT_1G: {
            CLOCK_EnableClock(s_enetClock[ENET_GetInstance(ENET_1G)]);
            ENET_SetSMI(ENET_1G,
                        CLOCK_GetRootClockFreq(kCLOCK_Root_Bus),
                        false);

            phy_rtl8211_resource.read  = mdio_read_rtl8211;
            phy_rtl8211_resource.write = mdio_write_rtl8211;

            break;
        }
        }

        (void)SILICONID_ConvertToMacAddr(&ethernet_config.macAddress);

        // --- IP4 config ---

        IP4_ADDR(&netif_address, address.a, address.b, address.c, address.d);
        IP4_ADDR(&netif_net_mask,
                 net_mask.a,
                 net_mask.b,
                 net_mask.c,
                 net_mask.d);

        IP4_ADDR(&netif_gateway_address,
                 gateway_address.a,
                 gateway_address.b,
                 gateway_address.c,
                 gateway_address.d);

        lwip_init();

        switch (ethernet_port) {

        case Port::PORT_100M:

            if (netif_add(&netif,
                          &netif_address,
                          &netif_net_mask,
                          &netif_gateway_address,
                          &ethernet_config,
                          ethernetif0_init,
                          ethernet_input) == NULL) {

                logger::errorf("Failed to set up net interface\r\n");
                return false;
            }
            break;

        case Port::PORT_1G:

            if (netif_add(&netif,
                          &netif_address,
                          &netif_net_mask,
                          &netif_gateway_address,
                          &ethernet_config,
                          ethernetif1_init,
                          ethernet_input) == NULL) {

                logger::errorf("Failed to set up net interface\r\n");
                return false;
            }
            break;
        }

        netif_set_default(&netif);
        netif_set_up(&netif);

        while (ethernetif_wait_linkup(&netif, 5000) != ERR_OK) {
            logger::errorf(
                "Ethernet auto-negotiation failed. Please check the cable "
                "connection and link partner setting.\r\n");
            return false;
        }

        logger::debugf("************************************************\r\n");
        logger::debugf(" IPv4 Address     : %u.%u.%u.%u\r\n",
                       ((u8_t*)&netif_address)[0],
                       ((u8_t*)&netif_address)[1],
                       ((u8_t*)&netif_address)[2],
                       ((u8_t*)&netif_address)[3]);
        logger::debugf(" IPv4 Subnet mask : %u.%u.%u.%u\r\n",
                       ((u8_t*)&netif_net_mask)[0],
                       ((u8_t*)&netif_net_mask)[1],
                       ((u8_t*)&netif_net_mask)[2],
                       ((u8_t*)&netif_net_mask)[3]);
        logger::debugf(" IPv4 Gateway     : %u.%u.%u.%u\r\n",
                       ((u8_t*)&netif_gateway_address)[0],
                       ((u8_t*)&netif_gateway_address)[1],
                       ((u8_t*)&netif_gateway_address)[2],
                       ((u8_t*)&netif_gateway_address)[3]);

        logger::debugf(" MAC address:     : %x:%x:%x:%x:%x:%x\r\n",
                       ethernet_config.macAddress[0],
                       ethernet_config.macAddress[1],
                       ethernet_config.macAddress[2],
                       ethernet_config.macAddress[3],
                       ethernet_config.macAddress[4],
                       ethernet_config.macAddress[5]);
        logger::debugf("************************************************\r\n");

        initialised = true;

        return true;
    }

    bool is_initialised() { return initialised; }

    void poll() {
        ethernetif_input(&netif);
        sys_check_timeouts();
    }

    ip4_addr_t* getAddress() { return &netif_address; }

    ip4_addr_t* getNetMask() { return &netif_net_mask; }

    ip4_addr_t* getGatewayAddress() { return &netif_gateway_address; }
}
