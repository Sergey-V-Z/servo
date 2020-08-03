/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "mb.h"
#include "mbport.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim5;

void saveFlash(void);
void readFlash(void);
bool calibration(uint32_t *start, uint32_t *finish);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// настройки адресов modbus
#define RECEIV_INPUT_START 0x0401 
#define RECEIV_INPUT_NREGS 8
#define REG_HOLDING_START 0x0000 
#define REG_HOLDING_NREGS 16
//настройки адресов регистров управления 
#define SLAVE_ADDRES_A 0
#define BOADRATE_A SLAVE_ADDRES_A + 1
#define COEFFICIENT_A BOADRATE_A + 1
#define POINTS_A COEFFICIENT_A + 1
#define STARTANGDLE_A POINTS_A + 1
#define FINASHANGLE_A STARTANGDLE_A + 1
#define CALCULATE_A FINASHANGLE_A + 1
#define POSITION_A CALCULATE_A + 1
#define SAVESETTINGS_A POSITION_A + 1
#define READSETTINGS_A SAVESETTINGS_A + 1
#define SETDEFOLT_A READSETTINGS_A + 1
#define CALIBRATION_A SETDEFOLT_A + 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static USHORT ReceivInputStart = RECEIV_INPUT_START;
static GPIO_PinState ReceivInputBuf[RECEIV_INPUT_NREGS];

//static USHORT usRegHoldingStart = REG_HOLDING_START;
//static USHORT usRegHoldingBuf[REG_HOLDING_NREGS] = {0};


uint16_t angle[512] = {0};
uint16_t ADC_Result[128] = {0};
uint8_t update = 1, tstart = 0, pos = 0, startCal = 0, setDef = 0, saveSettings = 0, readSettings = 0, stopCalibration = 0;
uint16_t halfPoints;
double totalAngle;
uint8_t convertFin = 0;

double K = 0.0391;                            // коэфицен, наулон кривой
uint32_t pointsROtate = 300;                  //количество точек для расчета кривой
uint32_t startAngle = 350, finishAngle = 650; // стартовый угол(L) в отсчетах ШпМ и финишный угол(R) в ШпМ
uint32_t startAngle_Cal = 0, finishAngle_Cal = 0; // стартовый угол(L) в отсчетах ШпМ и финишный угол(R) в ШпМ калибровочные данные
double min_U = 0.0807;                 // напряжение положения вала 
double max_U = 1.595;                  // напряжение положения вала 
uint32_t min_PosPWM = 0x00fa;          // ШпМ положения вала 
uint32_t max_PosPWM = 0x04e2;          // ШпМ положения вала 
double pos_U = 0;                      // текущее положение вала в вольтах
uint16_t posPWM = 0;                   // текущее положение вала в ШпМ 
uint32_t boarate = 115200;
uint32_t slaveAdr = 1;

/* USER CODE END Variables */
osThreadId mainTaskHandle;
osThreadId modBusHandle;
osTimerId ServoTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
double map(double x, double in_min, double in_max, double out_min, double out_max);

/* USER CODE END FunctionPrototypes */

void StartMainTask(void const * argument);
void ModBus_RTU(void const * argument);
void ServoSet(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    readFlash();
    
    if (pointsROtate == 0)
    {
      setDef = 0;
      
      K = 0.0391;
      pointsROtate = 300;
      startAngle = 350;
      finishAngle = 650;
      min_U = 0.0807;
      max_U = 1.595;
      boarate = 115200;
      slaveAdr = 1;
      min_PosPWM = 0x00fa;
      max_PosPWM = 0x04e2;
      startAngle_Cal = 0;
      finishAngle_Cal = 0;

      saveFlash();

    }   
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of ServoTimer */
  osTimerDef(ServoTimer, ServoSet);
  ServoTimerHandle = osTimerCreate(osTimer(ServoTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of mainTask */
  osThreadDef(mainTask, StartMainTask, osPriorityNormal, 0, 2048);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of modBus */
  osThreadDef(modBus, ModBus_RTU, osPriorityNormal, 0, 256);
  modBusHandle = osThreadCreate(osThread(modBus), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN StartMainTask */
  //HAL_ADC_Start(&hadc1);
  //HAL_TIM_Base_Start_IT(&htim5);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Result, 3);
//  saveFlash();
  
  /* Infinite loop */
  for(;;)
  {
    // обновить расчет кривой разгона торможения
    if (update) {
      update = 0;
      //проверка параметров
      if ((finishAngle_Cal != 0 & finishAngle_Cal!= 0xffffffff) && (startAngle_Cal != 0 & startAngle_Cal!= 0xffffffff) && 
        (pointsROtate != 0 & pointsROtate!= 0xffffffff) )
      {
        halfPoints = pointsROtate /2; // половина диапазона нужно для смещения
        totalAngle = (finishAngle_Cal - startAngle_Cal)/10; // максимальное значение

        for(int i = 0;i < pointsROtate+1;i++)
        {
          angle[i] = (uint16_t)((totalAngle/(1+exp(-K*((halfPoints * -1)+i))))*10)+startAngle_Cal; // расчет точки
        }
      }
    }
    
    // изменить позицию 
    if (tstart)
    {
      osTimerStart(ServoTimerHandle,5);
      tstart = 0;
    } 
    
    // если ацп завершил измерение то проведем расчеты
    if (convertFin == 1)
    {
      convertFin = 0;
      pos_U = ((double)ADC_Result[0] * VDD)/4095;
      posPWM = (uint16_t)map(pos_U, min_U, max_U, max_PosPWM, min_PosPWM);
    }
    
    // отколибровать положение вала по упорам
    if (startCal)
    {
      startCal = 0;
      calibration(&startAngle_Cal, &finishAngle_Cal);
			
			//проводим перерасчет	и сохраняем настройки
			//проверка параметров
      if ((finishAngle_Cal != 0 & finishAngle_Cal!= 0xffffffff) && (startAngle_Cal != 0 & startAngle_Cal!= 0xffffffff) && 
        (pointsROtate != 0 & pointsROtate!= 0xffffffff) )
      {
        halfPoints = pointsROtate /2; // половина диапазона нужно для смещения
        totalAngle = (finishAngle_Cal - startAngle_Cal)/10; // максимальное значение

        for(int i = 0;i < pointsROtate+1;i++)
        {
          angle[i] = (uint16_t)((totalAngle/(1+exp(-K*((halfPoints * -1)+i))))*10)+startAngle_Cal; // расчет точки
        }
				
				//Сохранить настройки во флеш
				saveSettings = 0;
				saveFlash();
				NVIC_SystemReset();

      }
			else{
				// Индикация ошибки параметров для расчета кривой
			}
    }
    
    // установить параметры пр умолчанию
    if (setDef)
    {
      setDef = 0;
      
      K = 0.0391;
      pointsROtate = 300;
      startAngle = 350;
      finishAngle = 650;
      min_U = 0.0807;
      max_U = 1.595;
      boarate = 115200;
      slaveAdr = 1;
      min_PosPWM = 0x00fa;
      max_PosPWM = 0x04e2;
      startAngle_Cal = 0;
      finishAngle_Cal = 0;

      saveFlash();
    }
    
    //Сохранить настройки
    if (saveSettings)
    {
      saveSettings = 0;
      saveFlash();
      NVIC_SystemReset();
    }
    
    //читать настройки
    if (readSettings)
    {
      readSettings = 0;
      readFlash();      
    }
		
		//функция доводчика
		
		switch (pos) // в зависимости от позиции выполняем доводку в нужную чторону
    { 
    	case 0:
				{
					if(HAL_GPIO_ReadPin(J4_GPIO_Port, J4_Pin) == GPIO_PIN_RESET)
					{
						osTimerStart(ServoTimerHandle,5);
					}
					break;
				}
    	case 1:
			{
				if(HAL_GPIO_ReadPin(J5_GPIO_Port, J5_Pin) == GPIO_PIN_RESET)
				{
					osTimerStart(ServoTimerHandle,5);
				}
    		break;
			}
    	default:
    		break;
    }
    osDelay(1);
  }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_ModBus_RTU */
/**
* @brief Function implementing the modBus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModBus_RTU */
void ModBus_RTU(void const * argument)
{
  /* USER CODE BEGIN ModBus_RTU */
  
  eMBErrorCode eStatus = eMBInit( MB_RTU, slaveAdr, 3, boarate, MB_PAR_NONE );
  eStatus = eMBEnable();

  /* Infinite loop */
  for(;;)
  {
    eMBPoll();
    osDelay(1);
  }
  /* USER CODE END ModBus_RTU */
}

/* ServoSet function */
void ServoSet(void const * argument)
{
  /* USER CODE BEGIN ServoSet */
	//Функция перемешение вала по расчитанной кривой
	
  static uint32_t i = 0, curentPosPWM, Closer_Coun;
	
	curentPosPWM = posPWM;
	
		// проверить изменилась ли позиция
	do{}while (convertFin != 1);
	convertFin = 0;
	pos_U = ((double)ADC_Result[0] * VDD)/4095;
	posPWM = (uint16_t)map(pos_U, min_U, max_U, max_PosPWM, min_PosPWM);

	// предыдущее положение вала вал, в движении движении
  switch (pos)
  {
    case 0: // проход 
    {	
      if ((posPWM < (curentPosPWM - HYSTERESIS)) | (posPWM > (curentPosPWM + HYSTERESIS)) | (i == 0) | (HAL_GPIO_ReadPin(J4_GPIO_Port, J4_Pin) == GPIO_PIN_RESET)) // при достижении конца массива отключаем шим и остонавливаем таймер
      {
				//если не дошли до концевика то доводим
				if(HAL_GPIO_ReadPin(J5_GPIO_Port, J5_Pin) == GPIO_PIN_SET)
				{
					TIM3->CCR1 = TIM3->CCR1 + 30;
					Closer_Coun++;
				}
				else{
					TIM3->CCR1 = 0;
					Closer_Coun = 0;
					osTimerStop(ServoTimerHandle);				
				}
        return;
      }
			else 
			{
				// устанавливаем следующее положение вала
				TIM3->CCR1 = angle[i];
				i--;	
			}

      break;
    }
    case 1:
    {	
      if ((posPWM < (curentPosPWM - HYSTERESIS)) | (posPWM > (curentPosPWM + HYSTERESIS)) | (i == pointsROtate+1) | (HAL_GPIO_ReadPin(J5_GPIO_Port, J5_Pin) == GPIO_PIN_RESET) ) // при достижении конца массива отключаем шим и остонавливаем таймер 
      {
        TIM3->CCR1 = 0;
        osTimerStop(ServoTimerHandle);
        //pos = 1;
        return;
      }
			// устанавливаем следующее положение вала
      TIM3->CCR1 = angle[i];
      i++; 
    break;
    }

  }
  
  /* USER CODE END ServoSet */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*description https://www.freemodbus.org/api/group__modbus__registers.html*/
//0x04
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= RECEIV_INPUT_START ) && ( usAddress + usNRegs <= RECEIV_INPUT_START + RECEIV_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - ReceivInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( ReceivInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( ReceivInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }			
	//HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
    }
    else
    {
	//HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}
//0x03 0x06 0x10
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    //int             iRegIndex;
    if (usAddress > 0)
    {usAddress--;}
    
    if(( (int)usAddress >= REG_HOLDING_START ) && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS )){
      //iRegIndex = ( int )( usAddress - usRegHoldingStart );
      
      if (eMode == MB_REG_READ)
      {
        switch (usAddress)
        {
          case SLAVE_ADDRES_A:
          {	
            uint16_t tmp = (uint16_t)slaveAdr;
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );

            break;
          }
          case BOADRATE_A:
          {	
            uint16_t tmp = (uint16_t)(boarate/10);
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );

            break;
          }
          case COEFFICIENT_A:
          {	
            uint16_t tmp = (uint16_t)(K*10000);
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );

            break;
          }
          case POINTS_A:
          {	
            uint16_t tmp = (uint16_t)pointsROtate;
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );

            break;
          }
          case STARTANGDLE_A:
          {	
            uint16_t tmp = (uint16_t)startAngle_Cal;
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );
            break;
          }
          case FINASHANGLE_A:
          {	
            uint16_t tmp = (uint16_t)finishAngle_Cal;
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );
            break;
          }
          case CALCULATE_A:
          {	
            eStatus = MB_EINVAL;
            break;
          }
          case POSITION_A:
          {	
            uint16_t tmp = (uint16_t)pos;
            *pucRegBuffer++ = ( unsigned char )( tmp >> 8 );
            *pucRegBuffer++ = ( unsigned char )( tmp & 0xFF );
            break;
          }
          case SAVESETTINGS_A:
          {	
            eStatus = MB_EINVAL;
            break;
          }
          case SETDEFOLT_A:
          {	
            eStatus = MB_EINVAL;
            break;
          }
          case CALIBRATION_A:
          {	
            eStatus = MB_EINVAL;
            break;
          }
          default:
          {	
            eStatus = MB_ENOREG;
            break;
          }
        }

      }
      else if(eMode == MB_REG_WRITE)
      {
        switch (usAddress)
        {
          case SLAVE_ADDRES_A:
          {	
            slaveAdr  = *pucRegBuffer++ << 8;
            slaveAdr |= (unsigned short)(*pucRegBuffer++);
            break;
          }
          case BOADRATE_A:
          {	
            uint16_t temp = 0;
            temp  = *pucRegBuffer++ << 8;
            temp |= (unsigned short)(*pucRegBuffer++);
            boarate = temp*10;
            break;
          }
          case COEFFICIENT_A:
          {	uint16_t temp = 0;
            temp  = *pucRegBuffer++ << 8;
            temp |= (unsigned short)(*pucRegBuffer++);
            K = (double)(temp/10000.0);
            break;
          }
          case POINTS_A:
          {	
            pointsROtate  = *pucRegBuffer++ << 8;
            pointsROtate |= (unsigned short)(*pucRegBuffer++);
            break;
          }
          case STARTANGDLE_A:
          {	
            startAngle_Cal  = *pucRegBuffer++ << 8;
            startAngle_Cal |= (unsigned short)(*pucRegBuffer++);
            break;
          }
          case FINASHANGLE_A:
          {	
            finishAngle_Cal  = *pucRegBuffer++ << 8;
            finishAngle_Cal |= (unsigned short)(*pucRegBuffer++);
            break;
          }
          case CALCULATE_A:
          {	
            update = 1;
            break;
          }
          case POSITION_A:
          {	
            if((startAngle_Cal !=0) & (finishAngle_Cal !=0) & (startAngle_Cal !=0xffffffff) & (finishAngle_Cal !=0xffffffff))
            {
              tstart = 1;
							pos = *(pucRegBuffer+1);
            }
            break;
          }
          case SAVESETTINGS_A:
          {	
            saveSettings = 1;
            break;
          }
          case READSETTINGS_A:
          {	
            readSettings = 1;
            break;
          }
          case SETDEFOLT_A:
          {	
            setDef = 1;
            break;
          }
          case CALIBRATION_A:
          {	
            uint16_t temp = 2;
            temp  = *pucRegBuffer++ << 8;
            temp |= (unsigned short)(*pucRegBuffer++);
            if (temp == 1)
            {
              startCal = 1;
              stopCalibration = 0;
            }
            else if(temp == 0)
            {
              startCal = 0;
              stopCalibration = 1;
            }
            break;
          }
          default:
          {	
            eStatus = MB_ENOREG;
            break;
          }
        }

      }
//      if(eMode == MB_REG_READ){
//        while( usNRegs > 0 )
//        {
//            *pucRegBuffer++ =
//                ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
//            *pucRegBuffer++ =
//                ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
//            iRegIndex++;
//            usNRegs--;
//          
//          // (startAngle_Cal !=0) & (finishAngle_Cal !=0) & (startAngle_Cal !=0xffffffff) & (finishAngle_Cal !=0xffffffff)
//        }

//      } else if(eMode == MB_REG_WRITE) {
//        while( usNRegs > 0 )
//        {
//            usRegHoldingBuf[iRegIndex]  = *pucRegBuffer++ << 8;
//            usRegHoldingBuf[iRegIndex] |= (unsigned short)(*pucRegBuffer++);
//            iRegIndex++;
//            usNRegs--;
//        }
//      }			
	//HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
    }
    else
    {
	//HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
        eStatus = MB_ENOREG;			
    }

    return eStatus;
}

// 0x01 0x0f 0x05
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    return MB_ENOREG;
}
//0x02
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}   


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  convertFin = 1;
  // вычисляем напряжение питания
  if(VDD == 0){
    VDD = (1.214 * 4095.0)/ADC_Result[1];
  }
}

/*calibration*/
bool calibration(uint32_t *start, uint32_t *finish)
{
  uint16_t addPos = STEP, curentPosPWM = posPWM;
 
  if ((posPWM > (min_PosPWM - HYSTERESIS)) & (posPWM < (min_PosPWM + HYSTERESIS)))
  {
    curentPosPWM = posPWM + 20;
    // включить шим на текушей позиции
    TIM3->CCR1 = curentPosPWM;

  } else if ((posPWM > (max_PosPWM - HYSTERESIS)) & (posPWM < (max_PosPWM + HYSTERESIS)))
  {
    curentPosPWM = posPWM - 20;
    // включить шим на текушей позиции
    TIM3->CCR1 = curentPosPWM;

  } else 
  {
    TIM3->CCR1 = posPWM;
  }
  osDelay(PAUSE_CAL);
  
  // прошупываем в плюс
  while(!(curentPosPWM >= max_PosPWM))
  { 
    // сдвинуть на шаг в лево
    curentPosPWM += addPos;
    TIM3->CCR1 = curentPosPWM;
    
    osDelay(PAUSE_CAL);

    // проверить изменилась ли позиция
    do{}while (convertFin != 1);
		convertFin = 0;
		pos_U = ((double)ADC_Result[0] * VDD)/4095;
		posPWM = (uint16_t)map(pos_U, min_U, max_U, max_PosPWM, min_PosPWM);

    // если posPWM отличается от curentPosPWM не больше чем на 5 то перейти к следуешему шагу иначе сохронить позицию и перейти к следуещей стороне
//    if ((posPWM < (curentPosPWM - HYSTERESIS)) | (posPWM > (curentPosPWM + HYSTERESIS)) | (posPWM >= max_PosPWM) | (stopCalibration))
//    {
//      TIM3->CCR1 = 0;
//      *finish = posPWM;
//      curentPosPWM = posPWM;
//      break;
//    }
			// test limit switch
			if (HAL_GPIO_ReadPin(J4_GPIO_Port, J4_Pin) == GPIO_PIN_SET)
			{
				TIM3->CCR1 = 0;
				*finish = posPWM;
				curentPosPWM = posPWM;
				break;
			}
    
    osDelay(PAUSE_CAL);
  }
    
  osDelay(PAUSE_CAL);
  
  // прошупываем в минус
  while(!(curentPosPWM <= min_PosPWM))
  { 
    // сдвинуть на шаг
    curentPosPWM -= addPos;
    TIM3->CCR1 = curentPosPWM;
    
    osDelay(PAUSE_CAL);
    
    // проверить изменилась ли позиция
    do{}while (convertFin != 1);
      convertFin = 0;
      pos_U = ((double)ADC_Result[0] * VDD)/4095;
      posPWM = (uint16_t)map(pos_U, min_U, max_U, max_PosPWM, min_PosPWM);

    // если posPWM отличается от curentPosPWM не больше чем на 5 то перейти к следуешему шагу иначе сохронить позицию и перейти к следуещей стороне
//    if ((posPWM < (curentPosPWM - HYSTERESIS)) | (posPWM > (curentPosPWM + HYSTERESIS)) | (posPWM <= min_PosPWM) | (stopCalibration))
//    {
//      stopCalibration = 0;
//      TIM3->CCR1 = 0;
//      *start = posPWM;
//      curentPosPWM = posPWM;
//      break;
//    }
			// test limit switch
			if (HAL_GPIO_ReadPin(J5_GPIO_Port, J5_Pin) == GPIO_PIN_SET)
			{
				TIM3->CCR1 = 0;
				*start = posPWM;
				curentPosPWM = posPWM;
				break;
			}    
    osDelay(PAUSE_CAL);  
}
  
  
  return true;
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void saveFlash(void)
{
  flash_unlock(); 
  flash_erase_page(K_Addres);
  
  flash_write_d(K_Addres, &K);
  flash_write(pointsROtate_Addres, pointsROtate);
  flash_write(startAngle_Addres, startAngle);
  flash_write(finishAngle_Addres, finishAngle);
  flash_write_d(min_U_Addres, &min_U);
  flash_write_d(max_U_Addres, &max_U);
  flash_write(boarate_Addres, boarate);
  flash_write(slaveAdr_Addres, slaveAdr);
  flash_write(min_PosPWM_Addres, min_PosPWM);
  flash_write(max_PosPWM_Addres, max_PosPWM);
  flash_write(startAngle_Cal_Addres, startAngle_Cal);
  flash_write(finishAngle_Cal_Addres, finishAngle_Cal);
  
  flash_lock();
}
void readFlash(void)
{
  flash_unlock(); 
  
  K = flash_read_d(K_Addres);
  pointsROtate = flash_read(pointsROtate_Addres);
  startAngle = flash_read(startAngle_Addres);
  finishAngle = flash_read(finishAngle_Addres);
  min_U = flash_read_d(min_U_Addres);
  max_U = flash_read_d(max_U_Addres);
  boarate = flash_read(boarate_Addres);
  slaveAdr = flash_read(slaveAdr_Addres);
  min_PosPWM = flash_read(min_PosPWM_Addres);
  max_PosPWM = flash_read(max_PosPWM_Addres);
  startAngle_Cal = flash_read(startAngle_Cal_Addres);
  finishAngle_Cal = flash_read(finishAngle_Cal_Addres);

  flash_lock();
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
