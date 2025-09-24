/*******************************************************************************
* \file mtb_display_co5300.c
*
* \brief
* Provides API implementation for the CO5300 amoled
* wearable display panel library.
*
********************************************************************************
* \copyright
* Copyright 2025 Cypress Semiconductor Corporation (an Infineon company)
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "mtb_display_co5300.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_LOW                        (0U)
#define GPIO_HIGH                       (1U)
#define NOMINAL_DELAY_MS                (20U)
#define INIT_DELAY_MS                   (60U)
#define RESET_DELAY_MS                  (100U)
#define NULL_VALUE                      (0x0U)
#define DSI_MAX_PACKET_SIZE             (16U)
#define DSI_CMD_INDEX                   (0U)
#define DSI_VAL_INDEX                   (1U)
#define DSI_CMD_LEN_1                   (1U)
#define DSI_CMD_LEN_2                   (2U)
#define DSI_CMD_LEN_5                   (5U)
#define MIPI_CONFIG_DEFAULT_VAL         (0U)
#define DISPLAY_RES_WIDTH               (512U)
#define DISPLAY_RES_HEIGHT              (466U)
#define MIPI_PIXEL_CLOCK_VAL            (14315U)
#define MIPI_NUM_LANES_ONE              (1U)
#define MIPI_NUM_LANES_TWO              (2U)
#define MIPI_PER_LANE_MBPS              (700U)
#define MIPI_MAXX_PHY_CLK               (2500000000UL)
#define MIPI_LP_MODE_ENABLE             (1U)
#define MIPI_LP_MODE_DISABLE            (0U)

/* CO5300 configuration register settings */
#define MTB_DISPLAY_UCS_CMD1            (0x00U)
#define MTB_DISPLAY_MCS_CMD2_PAGE_EXT   (0x20U)
#define MTB_DISPLAY_MCS_CMD2_OLED_IP    (0x80U)
#define MTB_DISPLAY_VREFP_VOLTAGE_VAL   (0x08U)
#define MTB_DISPLAY_OTP_CHKSUM_VAL1     (0x5AU)
#define MTB_DISPLAY_OTP_CHKSUM_VAL2     (0x59U)
#define MTB_DISPLAY_RGB888_FORMAT       (0x77U)
#define MTB_DISPLAY_TE_OFF              (0x00U)
#define MTB_DISPLAY_BCTRL_POS           (0x05U)
#define MTB_DISPLAY_DD_POS              (0x03U)
#define MTB_DISPLAY_MAX_BRIGHTNESS      (0xFFU)
#define MTB_DISPLAY_LANE_MODE_ONE       (0x00U)
#define MTB_DISPLAY_LANE_MODE_TWO       (0x01U)

/* Column start address parameters */
#define MTB_DISPLAY_CASET_PARAM1        (0x00U)
#define MTB_DISPLAY_CASET_PARAM2        (0x06U)
#define MTB_DISPLAY_CASET_PARAM3        (0x01U)
#define MTB_DISPLAY_CASET_PARAM4        (0xD7U)

/* Row start address parameters */
#define MTB_DISPLAY_RASET_PARAM1        (0x00U)
#define MTB_DISPLAY_RASET_PARAM2        (0x00U)
#define MTB_DISPLAY_RASET_PARAM3        (0x01U)
#define MTB_DISPLAY_RASET_PARAM4        (0xD1U)


/*******************************************************************************
* Global Variables
*******************************************************************************/
uint8_t packet[DSI_MAX_PACKET_SIZE];

cy_stc_mipidsi_display_params_t mtb_display_co5300_gfx_mipidsi_display_params =
{
    .pixel_clock    = MIPI_PIXEL_CLOCK_VAL,
    .hdisplay       = DISPLAY_RES_WIDTH,
    .vdisplay       = DISPLAY_RES_HEIGHT,
};
cy_stc_mipidsi_config_t mtb_display_co5300_gfx_mipi_dsi_config =
{
    .virtual_ch     = MIPI_CONFIG_DEFAULT_VAL,
    .num_of_lanes   = MIPI_NUM_LANES_ONE,
    .per_lane_mbps  = MIPI_PER_LANE_MBPS,
    .dpi_fmt        = CY_MIPIDSI_FMT_16BIT_24BPP_OP1,
    .dsi_mode       = DSI_COMMAND_MODE,
    .max_phy_clk    = MIPI_MAXX_PHY_CLK,
    .mode_flags     = VID_MODE_TYPE_BURST,
    .display_params = &mtb_display_co5300_gfx_mipidsi_display_params,
};


/*******************************************************************************
* Function Name: mtb_co5300_init
********************************************************************************
*
* This function performs the necessary initialization of AMOLED 1.43".
*
* \param *mipi_dsi_base
* Pointer to MIPI DSI instance
*
* \param *config
* Pointer to configuration data structure
*
* \return cy_en_mipidsi_status_t
* MIPI DSI operation status
*
*******************************************************************************/
cy_en_mipidsi_status_t mtb_display_co5300_init(GFXSS_MIPIDSI_Type* mipi_dsi_base,
                                               mtb_display_co5300_config_t* config)
{
    cy_en_mipidsi_status_t status = CY_MIPIDSI_SUCCESS;
    cy_stc_mipidsi_config_t* mipi_dsi_cfg = config->gfx_config->mipi_dsi_cfg;

    /* Display pin initialization sequence */
    /* Initialize RST pin with initial value - LOW */
    Cy_GPIO_Pin_FastInit(config->rst_port, config->rst_pin, CY_GPIO_DM_STRONG, GPIO_LOW,
                         HSIOM_SEL_GPIO);
    Cy_GPIO_Write(config->rst_port, config->rst_pin, GPIO_LOW);
    Cy_SysLib_Delay(RESET_DELAY_MS);

    /* VCI_EN set to 1.8V */
    Cy_GPIO_Pin_FastInit(config->vci_en_port, config->vci_en_pin, CY_GPIO_DM_STRONG, GPIO_HIGH,
                         HSIOM_SEL_GPIO);
    Cy_SysLib_Delay(RESET_DELAY_MS);

    /* RST - HIGH */
    Cy_GPIO_Write(config->rst_port, config->rst_pin, GPIO_HIGH);
    Cy_SysLib_Delay(RESET_DELAY_MS);

    Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_ENABLE);

    packet[0] = MTB_DISPLAY_VREFP_REG_C;
    packet[1] = MTB_DISPLAY_VREFP_VOLTAGE_VAL;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_CMD_PAGE_SWITCH;
    packet[1] = MTB_DISPLAY_MCS_CMD2_PAGE_EXT;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_OTP_CHKSUM1;
    packet[1] = MTB_DISPLAY_OTP_CHKSUM_VAL1;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_OTP_CHKSUM2;
    packet[1] = MTB_DISPLAY_OTP_CHKSUM_VAL2;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_CMD_PAGE_SWITCH;
    packet[1] = MTB_DISPLAY_MCS_CMD2_OLED_IP;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_LANESEL;
    packet[1] = (MIPI_NUM_LANES_ONE == mipi_dsi_cfg->num_of_lanes)
                ? MTB_DISPLAY_LANE_MODE_ONE : MTB_DISPLAY_LANE_MODE_TWO;

    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_CMD_PAGE_SWITCH;
    packet[1] = MTB_DISPLAY_UCS_CMD1;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_COLMOD;
    packet[1] = MTB_DISPLAY_RGB888_FORMAT;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_TEON;
    packet[1] = MTB_DISPLAY_TE_OFF;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_WRCTLRD;
    packet[1] = MTB_DISPLAY_MCS_CMD2_PAGE_EXT;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_SET_WRDISBV;
    packet[1] = MTB_DISPLAY_MAX_BRIGHTNESS;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

    packet[0] = MTB_DISPLAY_REG_CASET;
    packet[1] = MTB_DISPLAY_CASET_PARAM1;
    packet[2] = MTB_DISPLAY_CASET_PARAM2;
    packet[3] = MTB_DISPLAY_CASET_PARAM3;
    packet[4] = MTB_DISPLAY_CASET_PARAM4;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_5);

    packet[0] = MTB_DISPLAY_REG_RASET;
    packet[1] = MTB_DISPLAY_RASET_PARAM1;
    packet[2] = MTB_DISPLAY_RASET_PARAM2;
    packet[3] = MTB_DISPLAY_RASET_PARAM3;
    packet[4] = MTB_DISPLAY_RASET_PARAM4;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_5);

    packet[0] = MTB_DISPLAY_REG_SLPOUT;
    Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_1);
    Cy_SysLib_Delay(120);

    packet[0] = MTB_DISPLAY_REG_DISPON;
    status = Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_1);
    Cy_SysLib_Delay(30);

    Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_DISABLE);

    return status;
}


/*******************************************************************************
* Function Name: mtb_co5300_off
********************************************************************************
*
* This function turns of the AMOLED 1.43" display.
*
* \param *mipi_dsi_base
* Pointer to MIPI DSI instance
*
* \return cy_en_mipidsi_status_t
* MIPI DSI operation status
*
*******************************************************************************/
cy_en_mipidsi_status_t mtb_display_co5300_off(GFXSS_MIPIDSI_Type* mipi_dsi_base)
{
    cy_en_mipidsi_status_t status = CY_MIPIDSI_SUCCESS;

    status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_ENABLE);
    if (CY_MIPIDSI_SUCCESS == status)
    {
        /* Sending DISPOFF (Display OFF) command */
        packet[DSI_CMD_INDEX] = MTB_DISPLAY_REG_DISPOFF;
        packet[DSI_VAL_INDEX] = NULL_VALUE;
        status   = Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

        Cy_SysLib_Delay(NOMINAL_DELAY_MS);

        status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_DISABLE);
    }
    return status;
}


/*******************************************************************************
* Function Name: mtb_co5300_on
********************************************************************************
*
* This function turns on the AMOLED 1.43" display.
*
* \param *mipi_dsi_base
* Pointer to MIPI DSI instance
*
* \return cy_en_mipidsi_status_t
* MIPI DSI operation status
*
*******************************************************************************/
cy_en_mipidsi_status_t mtb_display_co5300_on(GFXSS_MIPIDSI_Type* mipi_dsi_base)
{
    cy_en_mipidsi_status_t status = CY_MIPIDSI_SUCCESS;

    status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_ENABLE);
    if (CY_MIPIDSI_SUCCESS == status)
    {
        /* Sending DISPON (Display ON) command */
        packet[DSI_CMD_INDEX] = MTB_DISPLAY_REG_DISPON;
        packet[DSI_VAL_INDEX] = NULL_VALUE;
        status   = Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

        Cy_SysLib_Delay(NOMINAL_DELAY_MS);

        status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_DISABLE);
    }

    return status;
}


/*******************************************************************************
* Function Name: mtb_co5300_set_brightness
********************************************************************************
*
* This function sets the brightness of AMOLED 1.43" display.
*
* \param *mipi_dsi_base
* Pointer to MIPI DSI instance
*
* \param value
* Brightness value
*
* \return cy_en_mipidsi_status_t
* MIPI DSI operation status
*
*******************************************************************************/
cy_en_mipidsi_status_t mtb_display_co5300_set_brightness(GFXSS_MIPIDSI_Type* mipi_dsi_base,
                                                         uint8_t value)
{
    cy_en_mipidsi_status_t status = CY_MIPIDSI_SUCCESS;

    status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_ENABLE);
    if (CY_MIPIDSI_SUCCESS == status)
    {
        /* Sending WRDISBV (Write Display Birghtness) command */
        packet[DSI_CMD_INDEX] = MTB_DISPLAY_REG_SET_WRDISBV;
        packet[DSI_VAL_INDEX] = (0xFF & value);
        status   = Cy_MIPIDSI_WritePacket(mipi_dsi_base, packet, DSI_CMD_LEN_2);

        Cy_SysLib_Delay(NOMINAL_DELAY_MS);

        status = Cy_MIPIDSI_CMD_MODE_DCS_SetLowPower(mipi_dsi_base, MIPI_LP_MODE_DISABLE);
    }

    return status;
}


/* [] END OF FILE */
