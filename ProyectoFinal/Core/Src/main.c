/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef enum {
    ESPERANDO_DEDO,
    MEDICION_EN_CURSO,
    RESULTADO_MOSTRADO
} EstadoSpO2;

EstadoSpO2 estado = ESPERANDO_DEDO;
uint32_t tiempo_inicio = 0;
#define REG_LED1_PA 0x0C // LED Rojo
#define REG_LED2_PA 0x0D // LED IR
#define MAX30102_ADDR         (0x57 << 1)
#define REG_PART_ID           0xFF
#define REG_MODE_CONFIG       0x09 // registros especificos del sensor
#define REG_SPO2_CONFIG       0x0A
#define REG_FIFO_WR_PTR       0x04
#define REG_FIFO_RD_PTR       0x06
#define REG_FIFO_DATA         0x07
#define REG_FIFO_OVERFLOW     0x05
extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef LeerMuestra();
HAL_StatusTypeDef LecturaSensor(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef EscrituraSensor(uint8_t reg, uint8_t value);
HAL_StatusTypeDef InicializacionSensor();
uint8_t fifo_data[6]; // 6 bytes que devuelve, 3 para ir y 3 para red
uint32_t ir_data = 0, red_data = 0;
float spo2_valor = 0.0f;
float CalcularSp02();
//void ConfigurarBluetooth();
void EnviarSpO2PorBluetooth(float spo2_valor);
uint32_t dedoDetectado = 1;
uint8_t estadoMensaje = 0;
float suma_spo2 = 0.0f;
uint16_t contador_muestras = 0;
int ultimo_segundo_mostrado = 11; // Empezamos en 11 para que muestre 10 la primera vez
uint32_t latidos_detectados = 0;
uint8_t pulso_anterior = 0; // 0: sin pulso alto, 1: pulso alto detectado

uint32_t tiempo_dedo_detectado = 0;  // tiempo en que se empezó a detectar el dedo
uint8_t dedo_estable = 0;            // flag para saber si ya estuvo el dedo 2 segundos
uint32_t tiempo_ultimo_latido = 0;
uint32_t valor_ir_min = 0xFFFFFFFF;
uint32_t valor_ir_max = 0;
#define UMBRAL_MINIMO 25000      // Ajustar según pruebas
#define TIEMPO_MIN_LATIDO 250    // ms entre latidos (máx ~200 bpm)
uint8_t msg_esperando_enviado = 0;
uint8_t primera_muestra = 1;  // flag global
uint8_t muestras_dedo_detectadas = 0;
#define MUESTRAS_CONSECUTIVAS_DEDO 5
uint32_t tiempo_boton_liberado = 0;  // Variable global
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //ConfigurarBluetooth();

  if (InicializacionSensor() != HAL_OK) {
      Error_Handler();
  }

 char start_msg[] = "Sensor MAX30102 inicializado.\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)start_msg, strlen(start_msg), 100);
  uint32_t tiempo_inicio = HAL_GetTick();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t estado_boton_actual = HAL_GPIO_ReadPin(GPIOA, BOTON_Pin);

	    if (estado_boton_actual == 0)  // BOTÓN APRETADO
	    {   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	        if (HAL_GetTick() - tiempo_boton_liberado < 3000) {
	            continue;  // Esperar estabilización después de encender
	        }

	        uint32_t tiempo_actual = HAL_GetTick();
	        uint32_t tiempo_transcurrido_ms = tiempo_actual - tiempo_inicio;
	        int segundos_restantes = 10 - (tiempo_transcurrido_ms / 1000);

	        if (LeerMuestra(&ir_data, &red_data) != HAL_OK) {
	            char msg[] = "Error leyendo sensor\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	            continue;
	        }

	        int dedo_presente = (ir_data > 45000 && red_data > 45000);

	        switch (estado)
	        {
	            case ESPERANDO_DEDO:
	                if (dedo_presente)
	                {
	                    muestras_dedo_detectadas++;

	                    if (muestras_dedo_detectadas >= MUESTRAS_CONSECUTIVAS_DEDO)
	                    {
	                        if (dedo_estable == 0)
	                        {
	                            tiempo_dedo_detectado = tiempo_actual;
	                            dedo_estable = 1;
	                        }
	                        else if ((tiempo_actual - tiempo_dedo_detectado) >= 3000)
	                        {
	                            char msg[] = "Dedo detectado, iniciando medicion...\r\n";
	                            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

	                            tiempo_inicio = tiempo_actual;
	                            suma_spo2 = 0.0f;
	                            contador_muestras = 0;
	                            latidos_detectados = 0;
	                            pulso_anterior = 0;
	                            ultimo_segundo_mostrado = 11;

	                            valor_ir_min = ir_data;
	                            valor_ir_max = ir_data;
	                            tiempo_ultimo_latido = 0;
	                            estado = MEDICION_EN_CURSO;
	                            muestras_dedo_detectadas = 0;
	                        }
	                    }
	                }
	                else
	                {
	                    muestras_dedo_detectadas = 0;

	                    if (dedo_estable == 1)
	                    {
	                        char msg[] = "Dedo perdido antes de iniciar medicion\r\n";
	                        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	                        dedo_estable = 0;
	                    }
	                    else
	                    {
	                        if (!msg_esperando_enviado)
	                        {
	                            char msg[] = "Esperando dedo...\r\n";
	                            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	                            msg_esperando_enviado = 1;
	                        }
	                    }
	                }
	                break;

	            case MEDICION_EN_CURSO:
	                if (!dedo_presente) {
	                    char msg[] = "Dedo perdido, cancelando medicion.\r\n";
	                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	                    estado = ESPERANDO_DEDO;
	                    dedo_estable = 0;
	                    msg_esperando_enviado = 0;
	                }
	                else {
	                    if (segundos_restantes > 0 && segundos_restantes != ultimo_segundo_mostrado) {
	                        char msg[50];
	                        int len = sprintf(msg, "Mantenga el dedo durante %d segundos...\r\n", segundos_restantes);
	                        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
	                        ultimo_segundo_mostrado = segundos_restantes;
	                    }

	                    float spo2_temp = CalcularSp02(ir_data, red_data);
	                    if (spo2_temp >= 50.0f && spo2_temp <= 100.0f) {
	                        suma_spo2 += spo2_temp;
	                        contador_muestras++;
	                    }

	                    valor_ir_min = (0.9f * valor_ir_min) + (0.1f * ir_data);
	                    valor_ir_max = (0.9f * valor_ir_max) + (0.1f * ir_data);

	                    uint32_t umbral = (valor_ir_min + valor_ir_max) / 2;
	                    static uint8_t pulso_detectado = 0;

	                    if (!pulso_detectado && ir_data > umbral && umbral > UMBRAL_MINIMO) {
	                        uint32_t tiempo_actual_latido = HAL_GetTick();
	                        if (tiempo_actual_latido - tiempo_ultimo_latido > TIEMPO_MIN_LATIDO) {
	                            latidos_detectados++;
	                            tiempo_ultimo_latido = tiempo_actual_latido;
	                            pulso_detectado = 1;
	                        }
	                    }
	                    else if (pulso_detectado && ir_data < umbral) {
	                        pulso_detectado = 0;
	                    }

	                    if (tiempo_transcurrido_ms >= 10000) {
	                        estado = RESULTADO_MOSTRADO;
	                    }
	                }
	                break;

	            case RESULTADO_MOSTRADO:
	                if (contador_muestras > 0) {
	                    float promedio_spo2 = suma_spo2 / contador_muestras;
	                    float bpm = (latidos_detectados * 60.0f) / 10.0f;

	                    char buffer[80];
	                    int len = sprintf(buffer, "Promedio SpO2: %.1f%% | Pulsaciones: %.1f BPM\r\n", promedio_spo2, bpm);
	                    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
	                }
	                else {
	                    char msg[] = "No se obtuvieron muestras validas.\r\n";
	                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	                }
	                estado = ESPERANDO_DEDO;
	                dedo_estable = 0;
	                break;
	        }

	        HAL_Delay(100);
	    }
	    else // BOTÓN LIBERADO
	    {   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	        if (estado != ESPERANDO_DEDO)
	        {
	            estado = ESPERANDO_DEDO;

	            dedo_estable = 0;
	            suma_spo2 = 0.0f;
	            contador_muestras = 0;
	            latidos_detectados = 0;
	            pulso_anterior = 0;
	            ultimo_segundo_mostrado = 11;
	            valor_ir_min = 0xFFFFFFFF;
	            valor_ir_max = 0;
	            ir_data = 0;
	            red_data = 0;
	            tiempo_ultimo_latido = 0;
	            msg_esperando_enviado = 0;
	            primera_muestra = 1;
	            char msg[] = "Boton liberado, medicion reiniciada\r\n";
	            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
	        }
	        tiempo_boton_liberado = HAL_GetTick();
	    }
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedEstado_GPIO_Port, LedEstado_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LedEstado_Pin */
  GPIO_InitStruct.Pin = LedEstado_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedEstado_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOTON_Pin */
  GPIO_InitStruct.Pin = BOTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Función para escribir un byte en un registro
HAL_StatusTypeDef EscrituraSensor(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// Función para leer un byte de un registro
HAL_StatusTypeDef LecturaSensor(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}
// Inicialización básica del sensor
HAL_StatusTypeDef InicializacionSensor(void) {
    uint8_t part_id;
    HAL_StatusTypeDef status;

    // Leer PART_ID
    status = LecturaSensor(REG_PART_ID, &part_id);
    if (status != HAL_OK || part_id != 0x15) return HAL_ERROR;

    // Resetear sensor
    EscrituraSensor(REG_MODE_CONFIG, 0x40);
    HAL_Delay(10);

    // Modo SpO2 (2 LEDs)
    EscrituraSensor(REG_MODE_CONFIG, 0x03);

    // Configurar SpO2: 100 Hz, ADC 18 bits
    EscrituraSensor(REG_SPO2_CONFIG, 0x27);

    // Limpiar punteros FIFO
    EscrituraSensor(REG_FIFO_WR_PTR, 0x00);
    EscrituraSensor(REG_FIFO_RD_PTR, 0x00);
    EscrituraSensor(REG_FIFO_OVERFLOW, 0x00);
    EscrituraSensor(REG_LED1_PA, 0x24);
    EscrituraSensor(REG_LED2_PA, 0x24);


    return HAL_OK;
}
HAL_StatusTypeDef LeerMuestra(uint32_t *ir, uint32_t *red) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDR, REG_FIFO_DATA, I2C_MEMADD_SIZE_8BIT, fifo_data, 6, 100);
    if (status != HAL_OK) return status;

    *ir = ((uint32_t)(fifo_data[0] & 0x03) << 16) | (fifo_data[1] << 8) | fifo_data[2];
    *red = ((uint32_t)(fifo_data[3] & 0x03) << 16) | (fifo_data[4] << 8) | fifo_data[5];

    return HAL_OK;
}

float CalcularSp02(uint32_t ir, uint32_t red) {
    if (ir == 0) return 0.0f;
    float ratio = ((float)ir) / ((float)red); //CHUSMEAR LA CONDICION
    float spo2 = 100.0f * ratio;

    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f) spo2 = 0.0f;

    return spo2;
}


//void ConfigurarBluetooth(void) {
//    // Probar comunicación AT
//    char at_test[] = "AT\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t*)at_test, strlen(at_test), 100);
//    HAL_Delay(200);
//
//    // Cambiar modo a esclavo
//    char at_role[] = "AT+ROLE=0\r\n";   // 0 = esclavo, 1 = maestro
//    HAL_UART_Transmit(&huart2, (uint8_t*)at_role, strlen(at_role), 100);
//    HAL_Delay(200);
//
//    // Cambiar el nombre del módulo
//    char at_name[] = "AT+NAME=MAX_SPO2\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t*)at_name, strlen(at_name), 100);
//    HAL_Delay(200);
//
//    // Configurar el baudrate a 115200, 1 stop bit, sin paridad
//    char at_uart[] = "AT+UART=115200,0,0\r\n";
//    HAL_UART_Transmit(&huart2, (uint8_t*)at_uart, strlen(at_uart), 100);
//    HAL_Delay(200);
//
//    // Configurar PIN si querés (opcional)
//    char at_pswd[] = "AT+PSWD=1234\r\n";   // Cambia el pin de emparejamiento
//    HAL_UART_Transmit(&huart2, (uint8_t*)at_pswd, strlen(at_pswd), 100);
//    HAL_Delay(200);
//}


void EnviarSpO2PorBluetooth(float spo2_valor) {
    char buffer[20];
    int length;

    // Convertir float a string, por ejemplo con 1 decimal
    length = sprintf(buffer, "SpO2: %.1f%%\r\n", spo2_valor);

    // Enviar por UART
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, length, 100);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
