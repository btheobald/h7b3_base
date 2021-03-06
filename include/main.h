#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);

extern unsigned char framebuffer_l8[272*480]; // Fixed Framebuffer

// LCD Control
#define LCD_DE_Pin LL_GPIO_PIN_7
#define LCD_DE_GPIO_Port GPIOK
#define LCD_HSYNC_Pin LL_GPIO_PIN_12
#define LCD_HSYNC_GPIO_Port GPIOI
#define LCD_CLK_Pin LL_GPIO_PIN_14
#define LCD_CLK_GPIO_Port GPIOI
#define LCD_VSYNC_Pin LL_GPIO_PIN_13
#define LCD_VSYNC_GPIO_Port GPIOI
#define LCD_INT_Pin LL_GPIO_PIN_2
#define LCD_INT_GPIO_Port GPIOH
#define LCD_BL_CTRL_Pin LL_GPIO_PIN_1
#define LCD_BL_CTRL_GPIO_Port GPIOA
#define LCD_ON_OFF_Pin LL_GPIO_PIN_2
#define LCD_ON_OFF_GPIO_Port GPIOA
// LCD Red
#define LCD_R0_Pin LL_GPIO_PIN_15
#define LCD_R0_GPIO_Port GPIOI
#define LCD_R1_Pin LL_GPIO_PIN_0
#define LCD_R1_GPIO_Port GPIOJ
#define LCD_R2_Pin LL_GPIO_PIN_1
#define LCD_R2_GPIO_Port GPIOJ
#define LCD_R3_Pin LL_GPIO_PIN_2
#define LCD_R3_GPIO_Port GPIOJ
#define LCD_R4_Pin LL_GPIO_PIN_3
#define LCD_R4_GPIO_Port GPIOJ
#define LCD_R5_Pin LL_GPIO_PIN_4
#define LCD_R5_GPIO_Port GPIOJ
#define LCD_R6_Pin LL_GPIO_PIN_5
#define LCD_R6_GPIO_Port GPIOJ
#define LCD_R7_Pin LL_GPIO_PIN_6
#define LCD_R7_GPIO_Port GPIOJ
// LCD Green
#define LCD_G0_Pin LL_GPIO_PIN_7
#define LCD_G0_GPIO_Port GPIOJ
#define LCD_G1_Pin LL_GPIO_PIN_8
#define LCD_G1_GPIO_Port GPIOJ
#define LCD_G2_Pin LL_GPIO_PIN_9
#define LCD_G2_GPIO_Port GPIOJ
#define LCD_G3_Pin LL_GPIO_PIN_10
#define LCD_G3_GPIO_Port GPIOJ
#define LCD_G4_Pin LL_GPIO_PIN_11
#define LCD_G4_GPIO_Port GPIOJ
#define LCD_G5_Pin LL_GPIO_PIN_0
#define LCD_G5_GPIO_Port GPIOK
#define LCD_G6_Pin LL_GPIO_PIN_1
#define LCD_G6_GPIO_Port GPIOK
#define LCD_G7_Pin LL_GPIO_PIN_2
#define LCD_G7_GPIO_Port GPIOK
// LCD Blue
#define LCD_B0_Pin LL_GPIO_PIN_12
#define LCD_B0_GPIO_Port GPIOJ
#define LCD_B1_Pin LL_GPIO_PIN_13
#define LCD_B1_GPIO_Port GPIOJ
#define LCD_B2_Pin LL_GPIO_PIN_14
#define LCD_B2_GPIO_Port GPIOJ
#define LCD_B3_Pin LL_GPIO_PIN_15
#define LCD_B3_GPIO_Port GPIOJ
#define LCD_B4_Pin LL_GPIO_PIN_3
#define LCD_B4_GPIO_Port GPIOK
#define LCD_B5_Pin LL_GPIO_PIN_4
#define LCD_B5_GPIO_Port GPIOK
#define LCD_B6_Pin LL_GPIO_PIN_5
#define LCD_B6_GPIO_Port GPIOK
#define LCD_B7_Pin LL_GPIO_PIN_6
#define LCD_B7_GPIO_Port GPIOK


#define WIFI_GPIO_Pin LL_GPIO_PIN_4
#define WIFI_GPIO_GPIO_Port GPIOI
#define SDNCAS_Pin LL_GPIO_PIN_15
#define SDNCAS_GPIO_Port GPIOG
#define I2S6_SDO_Pin LL_GPIO_PIN_14
#define I2S6_SDO_GPIO_Port GPIOG
#define OCSPI1_IO6_Pin LL_GPIO_PIN_9
#define OCSPI1_IO6_GPIO_Port GPIOG
#define OCSPI1_IO7_Pin LL_GPIO_PIN_7
#define OCSPI1_IO7_GPIO_Port GPIOD
#define D3_Pin LL_GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define SDIO1_D2_Pin LL_GPIO_PIN_10
#define SDIO1_D2_GPIO_Port GPIOC
#define WIFI_BOOT_Pin LL_GPIO_PIN_3
#define WIFI_BOOT_GPIO_Port GPIOI
#define WIFI_DATRDY_Pin LL_GPIO_PIN_5
#define WIFI_DATRDY_GPIO_Port GPIOI
#define FMC_NBL0_Pin LL_GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define USER_LED1_Pin LL_GPIO_PIN_11
#define USER_LED1_GPIO_Port GPIOG
#define SDIO1_CMD_Pin LL_GPIO_PIN_2
#define SDIO1_CMD_GPIO_Port GPIOD
#define SDIO1_CK_Pin LL_GPIO_PIN_12
#define SDIO1_CK_GPIO_Port GPIOC
#define JTCK_Pin LL_GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define uSD_Detect_Pin LL_GPIO_PIN_8
#define uSD_Detect_GPIO_Port GPIOI
#define FMC_NBL1_Pin LL_GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define JTDO_TRACESWO_Pin LL_GPIO_PIN_3
#define JTDO_TRACESWO_GPIO_Port GPIOB
#define I2S6_SDI_Pin LL_GPIO_PIN_12
#define I2S6_SDI_GPIO_Port GPIOG
#define D2_Pin LL_GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define JTDI_Pin LL_GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define SPI2_SCK_Pin LL_GPIO_PIN_12
#define SPI2_SCK_GPIO_Port GPIOA
#define SPI2_NSS_Pin LL_GPIO_PIN_11
#define SPI2_NSS_GPIO_Port GPIOA
#define I2S6_CK_Pin LL_GPIO_PIN_13
#define I2S6_CK_GPIO_Port GPIOG
#define SDIO1_D3_Pin LL_GPIO_PIN_11
#define SDIO1_D3_GPIO_Port GPIOC
#define WIFI_WKUP_Pin LL_GPIO_PIN_2
#define WIFI_WKUP_GPIO_Port GPIOI
#define WIFI_RST_Pin LL_GPIO_PIN_1
#define WIFI_RST_GPIO_Port GPIOI
#define JTMS_Pin LL_GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define VCP_RX_Pin LL_GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define SDIO1_D1_Pin LL_GPIO_PIN_9
#define SDIO1_D1_GPIO_Port GPIOC
#define WAKEUP_Pin LL_GPIO_PIN_13
#define WAKEUP_GPIO_Port GPIOC
#define VCP_TX_Pin LL_GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define SDIO1_D0_Pin LL_GPIO_PIN_8
#define SDIO1_D0_GPIO_Port GPIOC
#define SDCLK_Pin LL_GPIO_PIN_8
#define SDCLK_GPIO_Port GPIOG
#define A1_Pin LL_GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define A0_Pin LL_GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define MCO_Pin LL_GPIO_PIN_8
#define MCO_GPIO_Port GPIOA
#define OCSPI1_NCS_Pin LL_GPIO_PIN_6
#define OCSPI1_NCS_GPIO_Port GPIOG
#define A15_Pin LL_GPIO_PIN_5
#define A15_GPIO_Port GPIOG
#define AUDIO_NRST_Pin LL_GPIO_PIN_3
#define AUDIO_NRST_GPIO_Port GPIOG
#define A2_Pin LL_GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define A4_Pin LL_GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define A14_Pin LL_GPIO_PIN_4
#define A14_GPIO_Port GPIOG
#define USER_LED2_Pin LL_GPIO_PIN_2
#define USER_LED2_GPIO_Port GPIOG
#define A3_Pin LL_GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define A5_Pin LL_GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define OCSPI1_IO3_Pin LL_GPIO_PIN_6
#define OCSPI1_IO3_GPIO_Port GPIOF
#define OCSPI1_IO2_Pin LL_GPIO_PIN_7
#define OCSPI1_IO2_GPIO_Port GPIOF
#define SPI2_MISO_Pin LL_GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOC
#define OCSPI1_IO1_Pin LL_GPIO_PIN_9
#define OCSPI1_IO1_GPIO_Port GPIOF
#define I2C4_SDA_Pin LL_GPIO_PIN_13
#define I2C4_SDA_GPIO_Port GPIOD
#define D0_Pin LL_GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define D1_Pin LL_GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define D9_Pin LL_GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D13_Pin LL_GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define D15_Pin LL_GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define OCSPI1_IO0_Pin LL_GPIO_PIN_11
#define OCSPI1_IO0_GPIO_Port GPIOD
#define I2C4_SCL_Pin LL_GPIO_PIN_12
#define I2C4_SCL_GPIO_Port GPIOD
#define OCSPI1_IO4_Pin LL_GPIO_PIN_1
#define OCSPI1_IO4_GPIO_Port GPIOC
#define A7_Pin LL_GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define D4_Pin LL_GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D10_Pin LL_GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define SDNE1_Pin LL_GPIO_PIN_6
#define SDNE1_GPIO_Port GPIOH
#define SPI2_MOSI_Pin LL_GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOC
#define OCSPI1_IO5_Pin LL_GPIO_PIN_3
#define OCSPI1_IO5_GPIO_Port GPIOH
#define OCSPI1_DQS_Pin LL_GPIO_PIN_5
#define OCSPI1_DQS_GPIO_Port GPIOC
#define SDNRAS_Pin LL_GPIO_PIN_11
#define SDNRAS_GPIO_Port GPIOF
#define A9_Pin LL_GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define D11_Pin LL_GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define D7_Pin LL_GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D14_Pin LL_GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define I2S6_WS_Pin LL_GPIO_PIN_0
#define I2S6_WS_GPIO_Port GPIOA
#define SDNWE_Pin LL_GPIO_PIN_5
#define SDNWE_GPIO_Port GPIOH
#define A8_Pin LL_GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define A11_Pin LL_GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define D6_Pin LL_GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D12_Pin LL_GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define I2S6_MCK_Pin LL_GPIO_PIN_3
#define I2S6_MCK_GPIO_Port GPIOA
#define OCSPI1_CLK_Pin LL_GPIO_PIN_2
#define OCSPI1_CLK_GPIO_Port GPIOB
#define A6_Pin LL_GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A10_Pin LL_GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define D5_Pin LL_GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D8_Pin LL_GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define SDCKE1_Pin LL_GPIO_PIN_7
#define SDCKE1_GPIO_Port GPIOH
#define MCO1_Pin LL_GPIO_PIN_8
#define MCO1_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
