/*******************************************************************************
* \file mtb_display_co5300.h
*
* \brief
* Provides API declarations for the CO5300 display driver library.
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


#ifndef MTB_DISPLAY_CO5300_H
#define MTB_DISPLAY_CO5300_H


#if defined(__cplusplus)
extern "C" {
#endif


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_graphics.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* CO5300 register list */
#define MTB_DISPLAY_REG_SLPIN           (0x10U)
#define MTB_DISPLAY_REG_SLPOUT          (0x11U)
#define MTB_DISPLAY_REG_DISPOFF         (0x28U)
#define MTB_DISPLAY_REG_DISPON          (0x29U)
#define MTB_DISPLAY_REG_TEON            (0x35U)
#define MTB_DISPLAY_REG_COLMOD          (0x3AU)
#define MTB_DISPLAY_REG_CASET           (0x2AU)
#define MTB_DISPLAY_REG_RASET           (0x2BU)
#define MTB_DISPLAY_REG_SET_WRDISBV     (0x51U)
#define MTB_DISPLAY_REG_WRCTLRD         (0x53U)
#define MTB_DISPLAY_REG_WRHBMDISBV      (0x63U)
#define MTB_DISPLAY_REG_CMD_PAGE_SWITCH (0xFEU)
#define MTB_DISPLAY_VREFP_REG_C         (0x31U)
#define MTB_DISPLAY_REG_OTP_CHKSUM1     (0xF4U)
#define MTB_DISPLAY_REG_OTP_CHKSUM2     (0xF5U)
#define MTB_DISPLAY_REG_LANESEL         (0x03U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Structure for passing CO5300 parameter data */
typedef struct
{
    /* CO5300 pin data */
    GPIO_PRT_Type* rst_port;
    GPIO_PRT_Type* vci_en_port;
    uint32_t rst_pin;
    uint32_t vci_en_pin;
    cy_stc_gfx_config_t* gfx_config;
} mtb_display_co5300_config_t;

/* Graphics subsystem context for CO5300 */
extern cy_stc_mipidsi_display_params_t mtb_display_co5300_gfx_mipidsi_display_params;
extern cy_stc_mipidsi_config_t mtb_display_co5300_gfx_mipi_dsi_config;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_en_mipidsi_status_t mtb_display_co5300_init(GFXSS_MIPIDSI_Type* mipi_dsi_base,
                                               mtb_display_co5300_config_t* config);
cy_en_mipidsi_status_t mtb_display_co5300_off(GFXSS_MIPIDSI_Type* mipi_dsi_base);
cy_en_mipidsi_status_t mtb_display_co5300_on(GFXSS_MIPIDSI_Type* mipi_dsi_base);
cy_en_mipidsi_status_t mtb_display_co5300_set_brightness(GFXSS_MIPIDSI_Type* mipi_dsi_base,
                                                         uint8_t value);


#if defined(__cplusplus)
}
#endif /* __cplusplus */


#endif /* MTB_DISPLAY_CO5300_H */

/* [] END OF FILE */
