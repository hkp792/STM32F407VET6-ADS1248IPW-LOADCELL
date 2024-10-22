# STM32F407VET6-ADS1248IPW-LOADCELL
MY MAIN CHECKWEIGHER PROJECT
<br>
AUTHOR-HK Patel
#include "stm32f4xx_hal.h"

// Pin Definitions
#define CS_PIN     GPIO_PIN_5   // ADS1248 CS - PB5
#define DRDY_PIN   GPIO_PIN_1   // ADS1248 DRDY - PB1
#define RESET_PIN  GPIO_PIN_5   // ADS1248 RESET - PC5
#define START_PIN  GPIO_PIN_0   // ADS1248 START - PB0
#define SCLK_PIN   GPIO_PIN_5   // SPI SCLK - PA5
#define DIN_PIN    GPIO_PIN_7   // SPI MOSI - PA7
#define DOUT_PIN   GPIO_PIN_6   // SPI MISO - PA6

// LEDs and Switches
#define LED1_PIN   GPIO_PIN_13  // LED1 - PB13
#define LED2_PIN   GPIO_PIN_14  // LED2 - PB14
#define SW1_PIN    GPIO_PIN_15  // Switch 1 - PA15
#define SW2_PIN    GPIO_PIN_13  // Switch 2 - PC13

// RS485
#define MODBUS_MASTER_TX_PIN GPIO_PIN_11 // PB11
#define MODBUS_SLAVE_RX_PIN GPIO_PIN_9   // PA9
#define MODBUS_MASTER_DE_PIN GPIO_PIN_12 // PB12 for RS485 DE/RE
#define MODBUS_SLAVE_DE_PIN  GPIO_PIN_11 // RE11 for RS485 DE/RE

// MCP4725 DAC
#define DAC_SDA_PIN GPIO_PIN_7 // PB7
#define DAC_SCL_PIN GPIO_PIN_6 // PB6

// Flash memory
#define FLASH_CS_PIN GPIO_PIN_2  // PD2
#define FLASH_SCK_PIN GPIO_PIN_10 // PC10
#define FLASH_MOSI_PIN GPIO_PIN_12 // PC12
#define FLASH_MISO_PIN GPIO_PIN_11 // PC11

// GPIO Outputs
#define OUT1_PIN GPIO_PIN_0 // PC0
#define OUT2_PIN GPIO_PIN_1 // PC1
#define OUT3_PIN GPIO_PIN_2 // PC2
#define OUT4_PIN GPIO_PIN_3 // PC3
#define OUT5_PIN GPIO_PIN_4 // PC4

// GPIO Inputs
#define IN1_PIN GPIO_PIN_0 // PE0
#define IN2_PIN GPIO_PIN_1 // PE1
#define IN3_PIN GPIO_PIN_2 // PE2
#define IN4_PIN GPIO_PIN_3 // PE3
#define IN5_PIN GPIO_PIN_4 // PE4

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
void GPIO_Init(void);
void SPI_Init(void);
void UART_Init(void);
void I2C_Init(void);
void ADS1248_Init(void);
void Flash_Init(void);
void Modbus_Master_Init(void);
void Modbus_Slave_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    SPI_Init();
    UART_Init();
    I2C_Init();
    ADS1248_Init();
    Flash_Init();
    Modbus_Master_Init();
    Modbus_Slave_Init();

    while (1) {
        // Read switches and control LEDs
        if (HAL_GPIO_ReadPin(GPIOA, SW1_PIN) == GPIO_PIN_SET) {
            HAL_GPIO_TogglePin(GPIOB, LED1_PIN); // Toggle LED1
        }
        if (HAL_GPIO_ReadPin(GPIOC, SW2_PIN) == GPIO_PIN_SET) {
            HAL_GPIO_TogglePin(GPIOB, LED2_PIN); // Toggle LED2
        }
        
        // Example: set outputs based on inputs
        if (HAL_GPIO_ReadPin(GPIOE, IN1_PIN) == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(GPIOC, OUT1_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, OUT1_PIN, GPIO_PIN_RESET);
        }
        
        // Add more application code
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void GPIO_Init(void) {
    // Enable GPIO clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure ADC pins (CS, START, RESET)
    GPIO_InitStruct.Pin = CS_PIN | START_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RESET_PIN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure LED pins as output
    GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure switch pins as input
    GPIO_InitStruct.Pin = SW1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SW2_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure GPIO outputs
    GPIO_InitStruct.Pin = OUT1_PIN | OUT2_PIN | OUT3_PIN | OUT4_PIN | OUT5_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure GPIO inputs
    GPIO_InitStruct.Pin = IN1_PIN | IN2_PIN | IN3_PIN | IN4_PIN | IN5_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void SPI_Init(void) {
    // SPI1 configuration for ADS1248 and Flash
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

void UART_Init(void) {
    // UART2 configuration for debugging and Modbus
    __HAL_RCC_USART2_CLK_ENABLE();
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart2);

    // Modbus slave and master UART configuration
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
}

void I2C
