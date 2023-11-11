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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "aes.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN static const uint32_t pKeyAES[4] __ALIGN_END = {
                            0x00000000,0x00000000,0x00000000,0x00000000};

SUBGHZ_HandleTypeDef hsubghz;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_AES_Init(void);
static void MX_SUBGHZ_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RADIO_MODE_STANDBY_RC        0x02
#define RADIO_MODE_BITFIELD          0x70

uint8_t RadioCmdSetFreq[4] = {0b00110011, 0b10111100, 0b10100001, 0b00000000};	// 868,000,000Hz (868MHz)
uint8_t RadioResult = 0x00;
uint8_t RadioParam  = 0x00;
uint8_t RadioMode   = 0x00;
uint8_t RadioStatus = 0x00;

#define AES128 1
#define ECB 1

uint16_t RED_LED = GPIO_PIN_5;

char message[256];

uint8_t SUBGRF_ReadRegister( uint16_t addr )
{
    uint8_t data;
    HAL_SUBGHZ_ReadRegisters( &hsubghz, addr, &data, 1 );
    return data;
}


void SUBGRF_WriteRegister( uint16_t addr, uint8_t data )
{
    HAL_SUBGHZ_WriteRegisters( &hsubghz, addr, (uint8_t*)&data, 1 );
}

void SUBGRF_WriteCommand( SUBGHZ_RadioSetCmd_t Command, uint8_t *pBuffer,
                                        uint16_t Size )
{
    HAL_SUBGHZ_ExecSetCmd( &hsubghz, Command, pBuffer, Size );
 }

void SUBGRF_SetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];

    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SUBGRF_WriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}

void SUBGRF_SetTxParams( uint8_t paSelect, int8_t power, uint8_t rampTime )
{
    uint8_t buf[2];

    if( paSelect == 1 ) //1= RF0_LP (low power)
    {
        if( power == 15 )
        {
            SUBGRF_SetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            SUBGRF_SetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -17 )
        {
            power = -17;
        }
        SUBGRF_WriteRegister( 0x08E7, 0x18 ); // current max is 80 mA for the whole device [0x08E7=REG_OCP]
    }
    else // rfo_hp (2)
    {
        // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
        // RegTxClampConfig = @address 0x08D8
        SUBGRF_WriteRegister( 0x08D8, SUBGRF_ReadRegister( 0x08D8 ) | ( 0x0F << 1 ) ); //0x08D8=REG_TX_CLAMP
        // WORKAROUND END

        SUBGRF_SetPaConfig( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 )
        {
            power = 22;
        }
        else if( power < -9 )
        {
            power = -9;
        }
        SUBGRF_WriteRegister( 0x08E7, 0x38 ); // current max 160mA for the whole device [0x08E7=REG_OCP]
    }
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SUBGRF_WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

uint8_t SUBGRF_SetRfTxPower( int8_t power )
{
    uint8_t paSelect; //1=RFO_LP, 2=RFO_HP;

    int32_t TxConfig = 0; //1=RBI_GetTxConfig();

    switch (TxConfig)
    {
        case 0: //0=RBI_CONF_RFO_LP_HP:
        {
            if (power > 15)
            	{ paSelect = 2; } //2=RFO_HP
            else
            	{ paSelect = 1; } //1=RFO_LP
            break;
        }
        case 1: //1=RBI_CONF_RFO_LP
        {
            paSelect = 1; //1=RFO_LP
            break;
        }
        case 2: //2=RBI_CONF_RFO_HP
        {
            paSelect = 2; //2=RFO_HP
            break;
        }
        default:
            break;
    }

    SUBGRF_SetTxParams( paSelect, 0x16, 0x02 );//0x02=RADIO_RAMP_40_US

    return paSelect;
}






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    char commandChar[10] = "s"; //run zebra stripe test
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
  MX_AES_Init();
  MX_SUBGHZ_Init();
  MX_USART1_UART_Init();



  /* USER CODE BEGIN 2 */

	PrintRadioStatus();
  /* Set Sleep Mode */
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, &RadioParam, 1) != HAL_OK)
		{ ErrorOut("Failed Sleep mode"); }
	PrintRadioStatus();


	uint8_t retSetPower = 0;
	int8_t power = 22;
	retSetPower = SUBGRF_SetRfTxPower(power);
	PrintRadioStatus();









	/* Set Standby Mode */
	if (HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &RadioParam, 1) != HAL_OK)
		{ ErrorOut("Failed Standby mode"); }
	PrintRadioStatus();


	uint8_t radio_status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, RadioCmdSetFreq, 4);
	if ( radio_status != HAL_OK)
		{ ErrorOut("Failed Setting Radio Freq"); }
	PrintRadioStatus();


	/* Retrieve Status from SUBGHZ Radio */
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1) != HAL_OK)
		{ ErrorOut("Failed Get Radio Status"); }
	else
	{
		RadioMode   = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
		/* Check if SUBGHZ Radio is in RADIO_MODE_STANDBY_RC mode */
		if(RadioMode != RADIO_MODE_STANDBY_RC)
			{ ErrorOut("Radio not in standby mode"); }
	}




	/* Set Tx on */
	//radio_status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, NULL, 0);
	//if ( radio_status != HAL_OK)
	//	{ ErrorOut("Failed Setting Radio Continuous Wave"); }
	//PrintRadioStatus();

	/* Set Tx Mode with continuous wave */
	radio_status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSWAVE, NULL, 0);
	if ( radio_status != HAL_OK)
		{ ErrorOut("Failed Setting Radio Continuous Wave"); }
	PrintRadioStatus();

	/*uint8_t buf[4];
	buf[0]= 0x04;  	//set dutycycle
	buf[1]= 0x07;	//set output power to 14dBm
	buf[2]= 0;		//set HP PA
	buf[3]= 0x01; 	//random stuff that the doc wants
	if(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, buf, 4 )!=HAL_OK)
		{ ErrorOut("Failed Setting Radio PA CONFIG"); }


	buf[0]= 22; //set power according to table 35 in the Reference manual
	buf[1]= 0x02; //set rampup time to 40us
	if(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, buf, 2)!=HAL_OK)
		{ ErrorOut("Failed Setting Radio TX Params"); }
*/


	PrintMessage("Radio sending continuous wave");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  switch(commandChar[0]) {
	     case 'e'  :
	    	 SingleEncryption();
	         break;
	     case 's'  :
	    	 PrintMessage("Beginning zebra stripe test");
	    	 SurveySignal();
	         break;
	     case 'h'  :
	    	 PrintMessage("Usage:");
	    	 PrintMessage("  e: Single encryption");
	    	 PrintMessage("      set key followed by 128bit key in hex (e.g., 01 23 45 67 89 ab cd ef 01 12 13 14 15 16 17 18");
	    	 PrintMessage("  s: Survey Signals (Broadcasts constantly but performs periods of encryption and periods of NoOp)");
	    	 break;
	     case 'p':
	    	 //possible todo, implement power modification ability
	    	 /*
	    	  * //7. Setup PA
				buf[0]= 0x2;  	//set dutycycle
				buf[1]= 0x2;	//set output power to 14dBm
				buf[2]= 0;		//set HP PA
				buf[3]= 0x01; 	//random stuff that the doc wants
				if(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, buf, 4 )!=HAL_OK){Error_Handler();}

				//8. Define PA output power and rampup
				buf[0]= 0x16; //set power according to table 35 in the Reference manual
				buf[1]= 0x02; //set rampup time to 40us
				if(HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, buf, 2)!=HAL_OK){Error_Handler();}
	    	  */
	   	      HAL_UART_Receive(&huart1, (uint8_t *)commandChar, 10, 500000);
	   	      sprintf(message, "%s", commandChar);
	   	   	  PrintMessage(message);
	   	   	  PrintMessage("power output modification not implemented.");
	   	      break;
	     case 'k':
	    	  //possible todo: implement frequency modification
	   	      /*
	   	       *
				uint8_t radio_status = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, RadioCmdSetFreq, 4);
				if ( radio_status != HAL_OK)
					{ ErrorOut("Failed Setting Radio Freq"); }
			   *
	   	       */

	   	       break;
	     default :
	    	 break;
	  }
	  commandChar[0] = 'h';
	  PrintMessage("Waiting for next command");
	  HAL_UART_Receive(&huart1, (uint8_t *)commandChar, 1, 500000);
	  sprintf(message, "%s", commandChar);
	  PrintMessage(message);
    /* USER CODE END WHILE */
	  /*
	  //Heartbeat -- let us know you're alive
	  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_SET);
	  HAL_Delay(1000);

*/

	  //HAL_StatusTypeDef status = HAL_CRYP_Encrypt(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output, uint32_t Timeout);


/*
	  HAL_UART_Receive(&huart1, (uint8_t *)Rx_Data, 10, 5000);
	  strcpy(Tx_Data,Rx_Data);
	  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Data, 70, 5000);
*/


    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void PrintRadioStatus() {
	uint8_t get_status[32] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t radio_status = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &get_status, 32);
	uint8_t RadioMode   = 0x00;
	uint8_t RadioResult = 0x00;
	if ( radio_status != HAL_OK)
		{ ErrorOut("Failed Getting Radio Status"); }
	sprintf(message, "%x", get_status);
	PrintMessage(message);


	/* Retrieve Status from SUBGHZ Radio */
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_STATUS, &RadioResult, 1) != HAL_OK)
	{
		ErrorOut("Failed to check radio status");
	}
	else
	{
		/* Format Mode and Status receive from SUBGHZ Radio */
		RadioMode   = ((RadioResult & RADIO_MODE_BITFIELD) >> 4);
		sprintf(message, "Radio Mode %x", RadioMode);
		PrintMessage(message);
		sprintf(message, "Full Radio Mode %x", RadioResult);
		PrintMessage(message);
	}


	uint8_t buf[8];
	if (HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR, buf, 2) != HAL_OK) {
		ErrorOut("Failed to check error status");
	} else {
		/* Format Mode and Status receive from SUBGHZ Radio */
		sprintf(message, "Error Code %x", *buf);
		PrintMessage(message);
	}




}

void PrintMessage(char* message) {
	char newMessage[500];
	sprintf(newMessage, "%s\r\n", message);
	HAL_UART_Transmit(&huart1, newMessage, strlen(newMessage), 5000);
}

void ErrorOut(char* message) {
	PrintMessage(message);
	Error_Handler();
}



void SurveySignal() {
    uint8_t key[] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    uint8_t plaintext[]  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };

    uint32_t numIterations = 1000;
    uint32_t cryptoPerIteration = 300;
    uint32_t delayPerIteration = 2000; //in ms

    PrintMessage("Beginning to iterate between crypto and no-op...");
	for (int i=0; i<numIterations; i++) {
		PrintMessage("Encrypting...");
		for (int j=0; j<cryptoPerIteration; j++) {
			for (int k=0; k<16; k++)
				{ key[k] = rand() & 0xff; }
			struct AES_ctx ctx;
			AES_init_ctx(&ctx, key);
			AES_ECB_encrypt(&ctx, plaintext);
		}
		PrintMessage("Sleeping...");
		HAL_Delay(delayPerIteration);
	}
	PrintMessage("Completed All Signal Survey iterations");
}

void SingleEncryption () { //saveOrblankBit should be 0xff [random key] or 0x00 [static NULL key]
    srand((unsigned) time(NULL));
    uint8_t hexByte[2];
    uint8_t key[] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
    uint8_t plaintext[]  = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff };


    PrintMessage("Enter each byte in hex format. Do not enter spaces or press enter between or after entering bytes");
    for(uint8_t i=0; i<16;i++) {
    	HAL_UART_Receive(&huart1, (uint8_t *)hexByte, 2, 500000);
    	sscanf(hexByte, "%02x", &key[i]);
        sprintf(message, "%s", hexByte);
  	    PrintMessage(message);
    }

	//for (int i=0; i<16; i++)
	///	{ key[i] = rand() & 0xff & saveOrblankBit; } //To preserve timing, we still generate random bits even if we ultimately null them for a fixed key
	sprintf(message, "key: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", key[0],key[1],key[2],key[3],key[4],key[5],key[6],key[7],key[8],key[9],key[10],key[11],key[12],key[13],key[14],key[15]);
	PrintMessage(message);

	struct AES_ctx ctx;
	AES_init_ctx(&ctx, key);
	sprintf(message, "plaintext: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", plaintext[0],plaintext[1],plaintext[2],plaintext[3],plaintext[4],plaintext[5],plaintext[6],plaintext[7],plaintext[8],plaintext[9],plaintext[10],plaintext[11],plaintext[12],plaintext[13],plaintext[14],plaintext[15]);
	PrintMessage(message);
	AES_ECB_encrypt(&ctx, plaintext);
	sprintf(message, "crypttext: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", plaintext[0],plaintext[1],plaintext[2],plaintext[3],plaintext[4],plaintext[5],plaintext[6],plaintext[7],plaintext[8],plaintext[9],plaintext[10],plaintext[11],plaintext[12],plaintext[13],plaintext[14],plaintext[15]);
	PrintMessage(message);
	PrintMessage("Finished iteration");


		  //HAL_StatusTypeDef status = HAL_CRYP_Encrypt(CRYP_HandleTypeDef *hcryp, uint32_t *Input, uint16_t Size, uint32_t *Output, uint32_t Timeout);
		  //void AES_ECB_encrypt(const struct AES_ctx* ctx, uint8_t* buf);
	/*
		  HAL_UART_Receive(&huart1, (uint8_t *)Rx_Data, 10, 5000);
		  strcpy(Tx_Data,Rx_Data);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Tx_Data, 70, 5000);
	*/

}



void flashLight(uint8_t times) {
	  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_SET);
	  HAL_Delay(3000);
	  for (uint8_t i = 1; i <= times; ++i)
	  {
		  //Heartbeat -- let us know you're alive
		  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_RESET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_SET);
		  HAL_Delay(500);
	  }
	  HAL_GPIO_WritePin(GPIOB, RED_LED, GPIO_PIN_SET);
	  HAL_Delay(3000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief AES Initialization Function
  * @param None
  * @retval None
  */
static void MX_AES_Init(void)
{

  /* USER CODE BEGIN AES_Init 0 */

  /* USER CODE END AES_Init 0 */

  /* USER CODE BEGIN AES_Init 1 */

  /* USER CODE END AES_Init 1 */
  hcryp.Instance = AES;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeyAES;
  hcryp.Init.Algorithm = CRYP_AES_ECB;
  hcryp.Init.DataWidthUnit = CRYP_DATAWIDTHUNIT_WORD;
  hcryp.Init.HeaderWidthUnit = CRYP_HEADERWIDTHUNIT_WORD;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN AES_Init 2 */

  /* USER CODE END AES_Init 2 */

}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
static void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */
  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */
  //HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSWAVE, &pBuffer, Size);
  /* USER CODE END SUBGHZ_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
