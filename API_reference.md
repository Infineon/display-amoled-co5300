# CO5300 AMOLED DSI display driver library for ModusToolbox

## General description

A basic set of APIs to interact with the command mode AMOLED display panel. This library provides functions for supporting a 1.43 inch circular AMOLED wearable display driven by a CO5300 display driver.


## Code snippets

### Snippet 1: Display driver initialization as a part of graphics application

The following snippet initializes the display controller and then renders graphics artifacts using the display controller.

```
  #include "cybsp.h"
  #include "mtb_display_co5300.h"

  /*****************************************************************************
  * Macros
  *****************************************************************************/
  #define RESET_PORT       GPIO_PRT20
  #define RESET_PIN_NUM    7U
  #define VCI_EN_PORT      GPIO_PRT21
  #define VCI_EN_PIN_NUM   4U
  #define BRIGHTNESS_VALUE (127U)

  /*****************************************************************************
  * Variable(s)
  *****************************************************************************/
  GFXSS_Type* base = (GFXSS_Type*) GFXSS;
  cy_stc_gfx_context_t gfx_context;

  /* DC IRQ Configuration. */
  cy_stc_sysint_t dc_irq_cfg =
  {
      .intrSrc      = GFXSS_DC_IRQ,
      .intrPriority = 2U
  };

  /* GPU IRQ configuration. */
  cy_stc_sysint_t gpu_irq_cfg =
  {
      .intrSrc      = GFXSS_GPU_IRQ,
      .intrPriority = 2U
  };


  /*****************************************************************************
  * Function name: dc_irq_handler
  ******************************************************************************
  *
  * Display controller IRQ handler
  *
  *****************************************************************************/
  static void dc_irq_handler(void)
  {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      Cy_GFXSS_Clear_DC_Interrupt(GFXSS, &gfx_context);

      /* Way to synchronize frame transfer-based on DC interrupt. */
      xTaskNotifyFromISR(rtos_cm55_gfx_task_handle, 1, eSetValueWithOverwrite, 
                          &xHigherPriorityTaskWoken);

      /* Performs a context switch if a higher-priority task is woken. */
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }


  /*****************************************************************************
  * Function name: gpu_irq_handler
  ******************************************************************************
  *
  * GPU IRQ handler
  *
  *****************************************************************************/
  static void gpu_irq_handler(void)
  {
      Cy_GFXSS_Clear_GPU_Interrupt(GFXSS, &gfx_context);
      vg_lite_IRQHandler(); 
  }


  /*****************************************************************************
  * Code
  *****************************************************************************/

  int main(void)
  {
    cy_rslt_t result;
    cy_en_gfx_status_t status = CY_GFX_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    /* Enable global interrupts */
    __enable_irq();

    mtb_display_co5300_config_t co5300_config =
    {
        .gfx_config  = &GFXSS_config,
        .rst_port    = RESET_PORT,
        .rst_pin     = RESET_PIN_NUM,
        .vci_en_port = VCI_EN_PORT,
        .vci_en_pin  = VCI_EN_PIN_NUM,
    };

    /* MIPI-DSI display-specific configurations
     * If the display interface and MIPI DSI configuration for the panel 
     * have been set up using Device Configurator as outlined in the steps 
     * above, then this is optional.
     */
    GFXSS_config.mipi_dsi_cfg = &co5300_gfx_mipi_dsi_config;

    status = Cy_GFXSS_Init(base, &GFXSS_config, &gfx_context);

    if (CY_GFX_SUCCESS == status)
    {

      /* Initializes GFXSS DC interrupt. */
      sysint_status = Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);

      if (CY_SYSINT_SUCCESS != sysint_status)
      {
          CY_ASSERT(0);
      }

      /* Enables GFX DC interrupt in NVIC. */
      NVIC_EnableIRQ(GFXSS_DC_IRQ);

      /* Initializes GFX GPU interrupt. */
      sysint_status = Cy_SysInt_Init(&gpu_irq_cfg, gpu_irq_handler);

      if (CY_SYSINT_SUCCESS != sysint_status)
      {
          CY_ASSERT(0);
      }

      /* Enables GPU interrupt. */
      Cy_GFXSS_Enable_GPU_Interrupt(GFXSS);

      /* Enables GFX GPU interrupt in NVIC. */
      NVIC_EnableIRQ(GFXSS_GPU_IRQ);

      /* Initialize CO5300 AMOLED driver */
      status = mtb_display_co5300_init(GFXSS_GFXSS_MIPIDSI, &co5300_config);
      CY_ASSERT(CY_GFX_SUCCESS == status);

      /* Sets Video/Graphics layer buffer address */
      Cy_GFXSS_Set_FrameBuffer(base, (uint32_t*) render_target->address,
              &gfx_context);

      /* Transfer the frame buffer to display controller */
      Cy_GFXSS_Transfer_Frame(base, &gfx_context );

      /* Set display brightness */
      mipi_status = mtb_display_co5300_set_brightness(GFXSS_GFXSS_MIPIDSI, BRIGHTNESS_VALUE);
      if (CY_MIPIDSI_SUCCESS != mipi_status)
      {
          CY_ASSERT(0);
      }

      /* Switch OFF display */
      mipi_status = mtb_display_co5300_off(GFXSS_GFXSS_MIPIDSI);
      if (CY_MIPIDSI_SUCCESS != mipi_status)
      {
          CY_ASSERT(0);
      }

      /* Switch ON display */
      mipi_status = mtb_display_co5300_on(GFXSS_GFXSS_MIPIDSI);
      if (CY_MIPIDSI_SUCCESS != mipi_status)
      {
          CY_ASSERT(0);
      }
    }
    
    while (true);
  }
```

## Macros

```
MTB_DISPLAY_REG_SLEEP_IN       Sleep IN command register
MTB_DISPLAY_REG_SLEEP_OUT      Sleep OUT command register
MTB_DISPLAY_REG_DISPLAY_ON     Display ON command register
MTB_DISPLAY_REG_DISPLAY_OFF    Display OFF command register
MTB_DISPLAY_REG_SET_BRIGHTNESS Write Brightness command register
```

## Global variables
```
cy_stc_mipidsi_display_params_t co5300_gfx_mipidsi_display_params        MIPI DSI display parameters
cy_stc_mipidsi_config_t         co5300_gfx_mipi_dsi_config               MIPI DSI block configurations       
static const uint8_t            display_init_settings[][2]               Array to hold display controller registers and values
uint8_t                         packet[DSI_MAX_PACKET_SIZE]              MIPI DSI packet array
cy_en_mipidsi_status_t          status                                   MIPI DSI operation status
```

## Functions
cy_en_mipidsi_status_t `mtb_display_co5300_init(GFXSS_MIPIDSI_Type* mipi_dsi_base, mtb_display_co5300_config_t* config)`
>Performs initialization of the CO5300 display driver using MIPI DSI interface.

cy_en_mipidsi_status_t `mtb_display_co5300_off(GFXSS_MIPIDSI_Type* mipi_dsi_base)`
>Sends display off command to CO5300 display driver using MIPI DSI interface.

cy_en_mipidsi_status_t `mtb_display_co5300_on(GFXSS_MIPIDSI_Type* mipi_dsi_base)`
>Sends display on command to CO5300 display driver using MIPI DSI interface.

cy_en_mipidsi_status_t `mtb_display_co5300_set_brightness(GFXSS_MIPIDSI_Type* mipi_dsi_base, uint8_t value)`
>Sets brightness of specified value. 

## Macros documentation
Data type    | Values                         | Description     
 :-------    | :------------                  | :------------
 uint8_t     | MTB_DISPLAY_REG_SLEEP_IN       | Sleep IN command register
 uint8_t     | MTB_DISPLAY_REG_SLEEP_OUT      | Sleep OUT command register
 uint8_t     | MTB_DISPLAY_REG_DISPLAY_ON     | Display ON command register
 uint8_t     | MTB_DISPLAY_REG_DISPLAY_OFF    | Display OFF command register
 uint8_t     | MTB_DISPLAY_REG_SET_BRIGHTNESS | Write Brightness command register


## Function documentation
#### mtb_display_co5300_init
- cy_en_mipidsi_status_t `mtb_display_co5300_init(GFXSS_MIPIDSI_Type* mipi_dsi_base, mtb_display_co5300_config_t* config)`

> **Summary:** Performs initialization of the CO5300 display driver using MIPI DSI interface.
>
> **Parameter:**
>  Parameters            |  Description       
>  :-------              |  :------------
>  [in] mipi_dsi_base    |  Pointer to the MIPI DSI instance 
>  [in] config           |  Pointer to configuration data structure
>
> Return:
>  - cy_en_mipidsi_status_t       :         MIPI DSI operation status

#### mtb_display_co5300_off
- cy_en_mipidsi_status_t `mtb_display_co5300_off(GFXSS_MIPIDSI_Type* mipi_dsi_base)`

> **Summary:** Sends display OFF command to CO5300 display driver using MIPI DSI interface.
> 
> **Parameter:**
>  Parameters            |  Description       
>  :-------              |  :------------
>  [in] mipi_dsi_base    |  Pointer to the MIPI DSI instance
>
> Return:
>  - cy_en_mipidsi_status_t       :         MIPI DSI operation status

#### mtb_display_co5300_on
- cy_en_mipidsi_status_t `mtb_display_co5300_on(GFXSS_MIPIDSI_Type* mipi_dsi_base)`

> **Summary:** Sends display ON command to CO5300 display driver using MIPI DSI interface.
> 
> **Parameter:**
>  Parameters            |  Description       
>  :-------              |  :------------
>  [in] mipi_dsi_base    |  Pointer to the MIPI DSI instance
>
> Return:
>  - cy_en_mipidsi_status_t       :         MIPI DSI operation status

#### mtb_display_co5300_set_brightness
- cy_en_mipidsi_status_t `mtb_display_co5300_set_brightness(GFXSS_MIPIDSI_Type* mipi_dsi_base, uint8_t value)`

> **Summary:** Sets brightness of specified value.
> 
> **Parameter:**
>  Parameters            |  Description       
>  :-------              |  :------------
>  [in] mipi_dsi_base    |  Pointer to the MIPI DSI instance
>  [in] value            |  Brightness value in the range 0x0 to 0xFF
>
> Return:
>  - cy_en_mipidsi_status_t       :         MIPI DSI operation status

---
© 2025, Cypress Semiconductor Corporation (an Infineon company)
