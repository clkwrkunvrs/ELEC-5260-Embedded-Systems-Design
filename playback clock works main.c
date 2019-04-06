/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"



/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_audio.h"
#include "stm32l476g_discovery_qspi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define BUFFER_SIZE (uint32_t)0x0200 //may need to change this to 0x0100 //Is this where we select the # of
//...bytes for 20 seconds of audio?
#define WRITE_READ_ADDR     ((uint32_t)0x0050)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)
#define AUDIO_START_ADDRESS  (uint32_t)0x08020000
#define FREQ								(uint32_t)16000 //AUDIO FREQUENCY
#define VOLUME							(uint8_t)80

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint32_t msTicks;
uint8_t play = 0,idle = 1,pause = 0, sel = 0;
uint8_t playarray[4] = {0x50,0x4c,0x59,0x00};
uint8_t idlarray[4] = {0x49,0x44,0x4C,0x00};
uint8_t qspi_aRxBuffer[0x0200]; //let's put the whole darn audio file in here so i don't have to mess with a
//...circular buffer. It'll just be a very large buffer of 16-bit samples. Need to change to uint16_t?
uint8_t qspi_aTxBuffer[0x0200];

//make modes idle, record, and play
//idle = joyup
//record = joydown
//play = joyright
/*
1. Idle Mode: Enter this mode whenever the joystick UP button is pressed. This should halt any
recording or playback and reset any elapsed time to 0.
*optional: 2. *Record Mode: Enter this mode when the joystick DOWN button is pressed. When in this
mode, the joystick SEL button should start and stop recording audio data from the
MP34DT01 MEMS microphone. For the demonstration, record at least 20 seconds of audio
data.
3. Play Mode: Enter this mode when the joystick RIGHT button is pressed. When in this mode,
the joystick SEL button should start and stop playing audio data play through the CS43L22
Audio Codec and the headphone jack. For the demonstration, play at least 20 seconds of
audio data.
4. Audio data is to be written to the Quad SPI (QSPI) flash memory when in Record Mode,
and read from that flash memory during Play Mode.
5. On the LCD, display a 3-letter code indicating the current operating mode (record, play,
idle). Also display elapsed recording/play time in seconds (two digits). This should be 00 in
Idle Mode.
6. Use the SysTick timer to control sampling and playback rates (samples/second). To ensure
that the processor can keep up, you may keep the sampling rate relatively low, but it should
be high enough to produce �recognizable� audio.
*/

/*
Plan of execution:
I choose to implement play mode.
*1. Get the 3-digit code for each state (idle and play) working on the LCD since you've already worked with that. Use 'ply' and 'idl'
*1.1 Get the recording/play time showing on the lcd in seconds//this is ~done. Need to test in hardware
**2. Figure out how to get audio loaded into the Quad SPI flash memory. -answer, flash it to on-chip flash then write to qspi flash
3. Figure out how to Use the SysTick timer to control sampling and playback rates (samples/second)
*4. Tackle the handler for playback mode
*5. Write the playback timer to LCD
*6. Build the handler for idle mode
7. Realize that you just don't know what you don't know and take a walk before you get too frustrated
8. debug for 16 hours
9. Play 20 seconds of audio
9. Rejoice because you have a mostly completed project.
*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/


  //initialize everything
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_JOY_Init(JOY_MODE_EXTI);
	BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_Clear();
  //AUDIO_SAIPLLConfig(FREQ);
  //AUDIO_SAIx_Init(FREQ)
	//AUDIO_DFSDMx_Init(FREQ);
	BSP_QSPI_Init();
  BSP_QSPI_EnableMemoryMappedMode();
/*optional init functions:
  SystemHardwareInit();
 	BSP_AUDIO_IN_Init  ( uint32_t  AudioFreq, uint32_t  BitRes, uint32_t  ChnlNbr) //only if recording
  __HAL_RCC_CLEAR_RESET_FLAGS();
  */
  BSP_AUDIO_OUT_Init  ( OUTPUT_DEVICE_HEADPHONE, VOLUME,  FREQ  );
  //AUDIO_CODEC_Reset();
  SystemClock_Config();

	/* System clock functions in system_stm32l4xx.c (Startup)*/
	SystemCoreClockUpdate();
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);
	}
  //in case I only need interrupts every second?
  // if (SysTick_Config(SystemCoreClock)) {
	// 	while (1);
	// }
  /* USER CODE END Init */

  /* Configure the system clock */
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
//get stuff from on-chip flash and put a pointer to it in the qspi_aRxBuffer
//data address starts at 0x08020000 for on-chip flash
*qspi_aTxBuffer = ((uint8_t)AUDIO_START_ADDRESS); //0x08020000 //not sure this works;//i changed this from BUFFER_SIZE
BSP_QSPI_Write(qspi_aTxBuffer, WRITE_READ_ADDR, BUFFER_SIZE);//this'll take data from the tx buffer and write it to the qspi
//...gotta get the data written from the on-chip flash to the tx buffer.

/*other fcns i might need
uint8_t BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size);
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option); //how do I know when audio playback is done?
void BSP_AUDIO_OUT_RegisterCallbacks(Audio_CallbackTypeDef ErrorCallback,
                                     Audio_CallbackTypeDef HalfTransferCallback,
                                     Audio_CallbackTypeDef TransferCompleteCallback);
*/

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (play) {
    //character arrays passed in (ascii values at each index)
    BSP_QSPI_Read(qspi_aRxBuffer, WRITE_READ_ADDR, BUFFER_SIZE);//this should be finished//check on buffer size
    BSP_LCD_GLASS_DisplayString2(playarray);
    BSP_AUDIO_OUT_Play((uint16_t *)qspi_aRxBuffer, 320000);//rx buffer is uint8_t //pdata comes from SAI1 320,000 should be 8 second
    //...of audio
    }
    else if (idle) {
    BSP_LCD_GLASS_DisplayString2(idlarray);
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */

    }
	}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 8000000
  *            PLL_M                          = 1
  *            PLL_N                          = 20
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @retval None
  */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{/* oscillator and clocks configs */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};


	/* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();

  /* 80 Mhz from MSI 8Mhz */
  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 2000000
  *            HCLK(Hz)                       = 2000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 2000000
  *            PLL_M                          = 1
  *            PLL_N                          = 80
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @retval None
  */
void SystemLowClock_Config(void)
{
  /* oscillator and clocks configs */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t flatency = 0;

  /* Retrieve clock parameters */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &flatency );

  /* switch SYSCLK to MSI in order to modify PLL divider */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flatency) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Retrieve oscillator parameters */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* turn off PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
