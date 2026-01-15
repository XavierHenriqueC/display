/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * Versao 2.4: Implementação Modbus RTU as Slave (Henrique Xavier - jan/26)
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dmd.h"
#include "clk420ma_receiver.h"
#include "flash_sector.h"
#include "stdio.h"

// Modbus
#include "modbusSlave.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Definições display P10
#define DISPLAYS_ACROSS 2
#define DISPLAYS_DOWN 1
#define DISPLAY_X_OFFSET 3

// Definições serial
#define SERIAL_TIMEOUT 10000

// Definições de input pela serial
#define DISPLAY_CHANNEL 'C'
#define INPUT_MODE 'I'
#define TYPE_MODE 'T'
#define PROTOCOL 'R'
#define VOLTAGE 'V'
#define CURRENT 'A'
#define X_MIN 'x'
#define X_MAX 'X'
#define Y_MIN 'y'
#define Y_MAX 'Y'
#define DIGITAL 'D'
#define ANALOGIC 'A'
#define MODBUS 'M'
#define ASCII 'S'
#define DECIMAL_PLACE 'P'
#define ACQUIRE_TENSION_VALUE 'W'
#define ACQUIRE_CURRENT_VALUE 'Z'
#define FLASH_CHECK 'K'

// Definições RS485
// #define BUFFERS_RX_SIZE 24
#define BUFFERS_RX_SIZE 48

// Definições 4-20mA
#define NUM_READINGS 10 // Size of media filter

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Variaveis das configurações

char displayChannel = '1';
char inputMode = 'D';
char typeMode = 'A';
char protocol = 'M';
float yMin = 0.0f;
float xMax = 20.0f;
float xMin = 4.0f;
float yMax = 100.0f;
int decimalPlace = 1;
char msgConfigOn[15] = "CONFIG MODE ON";
char msgConfigOff[16] = "CONFIG MODE OFF";
char msgOk[3] = " Ok";

// Filtro media movel 4-20mA

float readings[NUM_READINGS]; // the readings from the analog input
int readIndex = 0;            // the index of the current reading
float total = 0.0f;           // the running total
float average = 0.0f;         // the average

float x, x_ant;

// Variaveis main

char inputString[BUFFERS_RX_SIZE];
uint8_t stringComplete = false;
uint8_t configMode = false;
uint8_t analogicStrSize;
uint8_t digitalStrSize;
uint32_t lastSerialInterrupt;

// Buffers Tx e RX - RS485 -> ASCII
uint8_t bufferRX[BUFFERS_RX_SIZE];
uint8_t bufferTX[BUFFERS_RX_SIZE];

// Struct para salvar os dados na flash
typedef struct
{
  char check; // caracter para checar se tem algo salvo na Flash
  char displayChannel_char;
  char inputMode_char;
  char typeMode_char;
  char protocol_char;
  float xMin_f;
  float xMax_f;
  float yMin_f;
  float yMax_f;
  uint8_t dPlace_int;
} DataFlashStruct;

DataFlashStruct dataFlash;
uint8_t bfdataFlash[sizeof(DataFlashStruct)];

// Variaveis de inicialização
uint8_t fw_init = 1;
uint32_t count_init = 0;
const char *INIT = "HFN SENSORS v2.4"; // String de inicialização
const char *IDLE = "- - - - - -";      // String de repouso

// Transformando float em string
char *reverse(char *str, int len)
{
  int start = 0;
  int end = len - 1;
  while (start < end)
  {
    char temp = str[start];
    str[start] = str[end];
    str[end] = temp;
    start++;
    end--;
  }
  return str;
}

void ftoa(float num, char *str, int decimalPlaces)
{
  int i = 0;
  int intPart = (int)num;
  float fracPart = num - intPart;
  if (intPart == 0)
  {
    str[i++] = '0';
  }
  else
  {
    while (intPart > 0)
    {
      str[i++] = intPart % 10 + '0';
      intPart /= 10;
    }
    str = reverse(str, i);
  }
  if (decimalPlaces > 0)
  {
    str[i++] = '.';
    for (int j = 0; j < decimalPlaces; j++)
    {
      fracPart *= 10;
      int digit = (int)fracPart;
      str[i++] = digit + '0';
      fracPart -= digit;
    }
  }
  str[i] = '\0';
}

// Funcao para calculo tonelada
double mapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
  float result = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
  return result;
}

// Interrupção Rx RS485
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  // MODBUS
  if (protocol == MODBUS)
  {
    if (RxData[0] == SLAVE_ID)
    {
      switch (RxData[1])
      {
      case 0x03:
        readHoldingRegs();
        break;
      case 0x10:
        writeHoldingRegs();
        break;
      default:
        modbusException(ILLEGAL_FUNCTION);
        break;
      }
    }

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
    return;
  }

  // ASCII
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, bufferRX, BUFFERS_RX_SIZE);
  for (int countBf = 0; countBf < BUFFERS_RX_SIZE; countBf++)
  {
    if ((char)bufferRX[countBf] == '\n')
    {
      lastSerialInterrupt = HAL_GetTick();
      stringComplete = true;
      break;
    }
    else
    {
      *(inputString + countBf) = (char)bufferRX[countBf];
    }
  }
}

// Função Tx RS485
void transmit_RS485(uint8_t *dataTx, uint16_t Size)
{

  // Enable to transmit
  HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RE_RS485_GPIO_Port, RE_RS485_Pin, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart1, dataTx, Size, 100);

  // Disable to transmit
  HAL_GPIO_WritePin(DE_RS485_GPIO_Port, DE_RS485_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE_RS485_GPIO_Port, RE_RS485_Pin, GPIO_PIN_RESET);
}

// Interrupção SCAN P10
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (fw_init == 1)
  {
    count_init += 1;
    if (count_init == 1400)
    {
      fw_init = 0;
      clearScreen(true);
      drawString(DISPLAY_X_OFFSET, 0, IDLE, strlen(IDLE), GRAPHICS_NORMAL);
    }
  }

  scanDisplayBySPI(&hspi2);
}

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Inicializando 4-20mA
  init_clk420mA(EN_420mA_GPIO_Port, EN_420mA_Pin);

  // Inicializando o display P10
  HAL_TIM_Base_Start_IT(&htim3);
  setFont(Arial_Black_16_ISO_8859_1);
  DMD(DISPLAYS_ACROSS, DISPLAYS_DOWN, OE_P10_GPIO_Port, OE_P10_Pin, SCLK_P10_GPIO_Port, SCLK_P10_Pin,
      A_P10_GPIO_Port, A_P10_Pin, B_P10_GPIO_Port, B_P10_Pin);

  // inicializando o array de filtro
  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++)
  {
    readings[thisReading] = 0;
  }

  // Inicializando os dados da EEPROM
  uint8_t *readFlash = (uint8_t *)ADDR_FLASH_SECTOR_4;
  dataFlash.check = (char)(*readFlash & 0xFF);

  if (dataFlash.check == FLASH_CHECK)
  {
    readFlash++;
    displayChannel = (char)(*readFlash & 0xFF);
    readFlash++;
    inputMode = (char)(*readFlash & 0xFF);
    readFlash++;
    typeMode = (char)(*readFlash & 0xFF);
    readFlash++;
    protocol = (char)(*readFlash & 0xFF);
    readFlash++;
    xMin = *((float *)readFlash);
    readFlash += sizeof(float);
    xMax = *((float *)readFlash);
    readFlash += sizeof(float);
    yMin = *((float *)readFlash);
    readFlash += sizeof(float);
    yMax = *((float *)readFlash);
    readFlash += sizeof(float);
    decimalPlace = (*readFlash & 0xFF);

    dataFlash.displayChannel_char = displayChannel;
    dataFlash.inputMode_char = inputMode;
    dataFlash.typeMode_char = typeMode;
    dataFlash.protocol_char = protocol;
    dataFlash.xMin_f = xMin;
    dataFlash.xMax_f = xMax;
    dataFlash.yMin_f = yMin;
    dataFlash.yMax_f = yMax;
    dataFlash.dPlace_int = decimalPlace;
  }
  else
  {
    dataFlash.check = FLASH_CHECK;
    dataFlash.displayChannel_char = displayChannel;
    dataFlash.inputMode_char = inputMode;
    dataFlash.typeMode_char = typeMode;
    dataFlash.protocol_char = protocol;
    dataFlash.xMin_f = xMin;
    dataFlash.xMax_f = xMax;
    dataFlash.yMin_f = yMin;
    dataFlash.yMax_f = yMax;
    dataFlash.dPlace_int = decimalPlace;
    memcpy(bfdataFlash, &dataFlash, sizeof(DataFlashStruct));
    Flash_Write_Data(ADDR_FLASH_SECTOR_4, (uint32_t *)bfdataFlash, sizeof(DataFlashStruct));
  }

  // Inicializando RS485
  MX_USART1_UART_Init();

  if (protocol == MODBUS)
  {
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
  }
  else
  {
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, bufferRX, BUFFERS_RX_SIZE);
  }

  // Variaveis display p10
  uint32_t start = HAL_GetTick();
  uint32_t timer = start;

  drawMarquee(INIT, strlen(INIT), (32 * DISPLAYS_ACROSS) - 1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (fw_init)
    {
      if ((timer + 30) < HAL_GetTick())
      {
        stepMarquee(-1, 0);
        timer = HAL_GetTick();
      }
    }
    else
    {
      uint32_t currentTime = HAL_GetTick();
      if (currentTime - lastSerialInterrupt >= SERIAL_TIMEOUT && inputMode == DIGITAL)
      {
        drawString(DISPLAY_X_OFFSET, 0, IDLE, strlen(IDLE), GRAPHICS_NORMAL);
      }
      if (stringComplete == true)
      {
        if (strstr(inputString, "CONFIG") != NULL)
        {
          if (inputString[7] == '1')
          {
            configMode = true;
            drawString(DISPLAY_X_OFFSET, 0, IDLE, strlen(IDLE), GRAPHICS_NORMAL);
            transmit_RS485((uint8_t *)msgConfigOn, strlen(msgConfigOn));
          }
          else
          {
            configMode = false;
            transmit_RS485((uint8_t *)msgConfigOff, strlen(msgConfigOff));
          }
        }

        // MODO DIGIAL
        if (!configMode)
        {
          if (inputMode == DIGITAL)
          {

            if (protocol == MODBUS)
            {
              if (Holding_Registers_Database[0] <= 65535 && Holding_Registers_Database[1] <= 65535 && Holding_Registers_Database[2] <= 65535)
              {
                int modbusCasasDecimais = (int)Holding_Registers_Database[0];
                float valueFloat = modbus_u16_to_float32_be(Holding_Registers_Database[1], Holding_Registers_Database[2]);

                if ((timer + 2000) < HAL_GetTick())
                {

                  if (valueFloat < 10)
                  {
                    digitalStrSize = 2 + modbusCasasDecimais;
                  }
                  else if (valueFloat < 100)
                  {
                    digitalStrSize = 3 + modbusCasasDecimais;
                  }
                  else if (valueFloat < 1000)
                  {
                    digitalStrSize = 4 + modbusCasasDecimais;
                  }
                  else
                  {
                    digitalStrSize = 5 + modbusCasasDecimais;
                  }

                  char tempString[9];
                  memset(tempString, ' ', sizeof(tempString));
                  ftoa(valueFloat, tempString, modbusCasasDecimais);
                  tempString[digitalStrSize] = 't';
                  drawString(DISPLAY_X_OFFSET, 0, tempString, sizeof(tempString), GRAPHICS_NORMAL);
                  timer = HAL_GetTick();
                }
              }
            }
            else
            {
              // ASCII - EXISTENTE
              if (inputString[0] == displayChannel && inputString[1] == '=')
              {
                for (int count = 0; count < strlen(inputString) - 2; count++)
                {
                  if (inputString[2 + count] == '.')
                  {
                    digitalStrSize = 3 + count + decimalPlace;
                    break;
                  }
                }
                memset(&(inputString[digitalStrSize + 1]), 32, sizeof(inputString) - digitalStrSize);
                inputString[digitalStrSize] = ' t';
                drawString(DISPLAY_X_OFFSET, 0, &inputString[2], strlen(inputString) - 2, GRAPHICS_NORMAL);
              }
            }
          }
        }
        else
        {

          if (strstr(inputString, "GET") != NULL)
          {
            switch (inputString[4])
            {
            case DISPLAY_CHANNEL:
              bufferTX[0] = (uint8_t)displayChannel;
              transmit_RS485(bufferTX, 1);
              break;
            case INPUT_MODE:
              bufferTX[0] = (uint8_t)inputMode;
              transmit_RS485(bufferTX, 1);
              break;
            case TYPE_MODE:
              bufferTX[0] = (uint8_t)typeMode;
              transmit_RS485(bufferTX, 1);
              break;
            case PROTOCOL:
              bufferTX[0] = (uint8_t)protocol;
              transmit_RS485(bufferTX, 1);
              break;
            case X_MIN:
              ftoa(xMin, (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            case X_MAX:
              ftoa(xMax, (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            case Y_MIN:
              ftoa(yMin, (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            case Y_MAX:
              ftoa(yMax, (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            case DECIMAL_PLACE:
              sprintf((char *)&bufferTX, "%d", decimalPlace);
              transmit_RS485(bufferTX, 1);
              break;
            case ACQUIRE_CURRENT_VALUE:
              ftoa(read_clk420mA(&hspi1, CS_420mA_GPIO_Port, CS_420mA_Pin), (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            case ACQUIRE_TENSION_VALUE:
              ftoa(read_clk420mA_voltage(&hspi1, CS_420mA_GPIO_Port, CS_420mA_Pin), (char *)bufferTX, 3);
              transmit_RS485(bufferTX, 10);
              break;
            }
          }
          else if (strstr(inputString, "SET") != NULL)
          {
            switch (inputString[4])
            {
            case DISPLAY_CHANNEL:
              displayChannel = inputString[6];
              dataFlash.displayChannel_char = displayChannel;
              break;
            case INPUT_MODE:
              inputMode = inputString[6];
              dataFlash.inputMode_char = inputMode;
              break;
            case TYPE_MODE:
              typeMode = inputString[6];
              dataFlash.typeMode_char = typeMode;
              break;
            case PROTOCOL:
              protocol = inputString[6];
              dataFlash.protocol_char = protocol;
              break;
            case X_MIN:
              xMin = atof(&inputString[6]);
              dataFlash.xMin_f = xMin;
              break;
            case X_MAX:
              xMax = atof(&inputString[6]);
              dataFlash.xMax_f = xMax;
              break;
            case Y_MIN:
              yMin = atof(&inputString[6]);
              dataFlash.yMin_f = yMin;
              break;
            case Y_MAX:
              yMax = atof(&inputString[6]);
              dataFlash.yMax_f = yMax;
              break;
            case DECIMAL_PLACE:
              decimalPlace = (uint8_t)(inputString[6] - '0');
              dataFlash.dPlace_int = decimalPlace;
              break;
            }
            bufferTX[0] = inputString[4];
            transmit_RS485(bufferTX, 1);
            transmit_RS485((uint8_t *)msgOk, strlen(msgOk));
            dataFlash.check = FLASH_CHECK;

            // Enviando todos os dados para a flash
            if (bufferTX[0] == DECIMAL_PLACE)
            {
              memcpy(bfdataFlash, &dataFlash, sizeof(DataFlashStruct));
              Flash_Write_Data(ADDR_FLASH_SECTOR_4, (uint32_t *)bfdataFlash, sizeof(DataFlashStruct));
              transmit_RS485((uint8_t *)msgOk, strlen(msgOk));
            }
          }
        }
        memset(inputString, 0x0, sizeof(inputString));
      }

      // MODO ANALOGICO
      if (!configMode)
      {
        if (inputMode == ANALOGIC)
        {

          if (typeMode == CURRENT)
          { // CORRENTE
            x = read_clk420mA(&hspi1, CS_420mA_GPIO_Port, CS_420mA_Pin);
            if (x == 255 || x == 0)
            {
              // erro de leitura
              x = x_ant;
            }
            else
            {
              x_ant = x;
            }
          }
          else
          { // TENSAO
            x = read_clk420mA_voltage(&hspi1, CS_420mA_GPIO_Port, CS_420mA_Pin);
          }
          float value = mapValue(x, xMin, xMax, yMin, yMax);

          // Aplicando a média
          total = total - readings[readIndex];
          readings[readIndex] = value;
          total = total + readings[readIndex];
          readIndex = readIndex + 1;
          if (readIndex >= NUM_READINGS)
          {
            readIndex = 0;
          }

          value = total / NUM_READINGS;

          if ((timer + 2000) < HAL_GetTick())
          {

            if (value < 10)
            {
              analogicStrSize = 2 + decimalPlace;
            }
            else if (value < 100)
            {
              analogicStrSize = 3 + decimalPlace;
            }
            else if (value < 1000)
            {
              analogicStrSize = 4 + decimalPlace;
            }
            else
            {
              analogicStrSize = 5 + decimalPlace;
            }

            char tempString[9];
            memset(tempString, ' ', sizeof(tempString));
            ftoa(value, tempString, decimalPlace);
            tempString[analogicStrSize] = 't';
            drawString(DISPLAY_X_OFFSET, 0, tempString, sizeof(tempString), GRAPHICS_NORMAL);
            timer = HAL_GetTick();
          }
        }
      }
    }
  }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 64;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, EN_420mA_Pin|CS_420mA_Pin|B_P10_Pin|A_P10_Pin
                          |SCLK_P10_Pin|DE_RS485_Pin|RE_RS485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OE_P10_GPIO_Port, OE_P10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_420mA_Pin CS_420mA_Pin B_P10_Pin A_P10_Pin
                           SCLK_P10_Pin DE_RS485_Pin RE_RS485_Pin */
  GPIO_InitStruct.Pin = EN_420mA_Pin|CS_420mA_Pin|B_P10_Pin|A_P10_Pin
                          |SCLK_P10_Pin|DE_RS485_Pin|RE_RS485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OE_P10_Pin */
  GPIO_InitStruct.Pin = OE_P10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OE_P10_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
