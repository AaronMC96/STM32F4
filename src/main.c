/**
 ******************************************************************************
 * @file    main.c
 * @author  Ejercicio Practico de Laboratorio
 * @version V1.0.0
 * @date    1-October-2020
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/**
  ******************************************************************************
  * @file    main.c
  * @title	 Ejercicio 1 - Practica de Problemas N°3.
  * @author  Apaza Adolfo.
  * @date    1-Junio-2020
  * @brief   Este programa implementa el manejo del ADC2 con detección de fin
  * de conversión por polling.
  * Permite que el usuario inyecte un nivel de tensión analogico de entre 0 y
  * 3V en el pin PC2(ADC2).
  * El valor adquirido de la señal del Pin PC2 es mostrado en la segunda linea del display 16x02,
  * como valor de tension, y el valor cuentas obtenidas es mostrado en la primer linea.
  *
  * Por detalles de funcionamiento ver:
  * - STM32f407 - DM00037051.pdf
  * - Manual de referencia STM32F4XXXX - ARM 32 bits - DM00031020.pdf
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <serial_debug.h>
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "httpd.h"
#include <stdint.h>
#include "udp_echoserver.h"


/* Kernel includes. */
//#include "stm32f4xx.h"
#include "../FreeRTOS/include/FreeRTOS.h"
#include "../FreeRTOS/include/queue.h"
#include "../FreeRTOS/include/semphr.h"
#include "../FreeRTOS/include/task.h"
#include "../FreeRTOS/include/timers.h"
#include <stm32f4xx_adc.h>
#include "adc.h"

//--------------------------------------------------------------
//Define los delay de las tareas a ejecutar, en ms
//--------------------------------------------------------------
#define DelayRED		1000
#define DelayBLUE		500
#define DelayORANGE		200
#define DelayDP			10
//--------------------------------------------------------------

RCC_ClocksTypeDef RCC_Clocks;

//--------------------------------------------------------------
// Prototipos de las tareas del FreeRTOS
//--------------------------------------------------------------
void BlinkRED(void *pvParameters);
void BlinkBLUE(void *pvParameters);
void BlinkORANGE(void *pvParameters);
void Lee_DP(void *pvParameters);
//--------------------------------------------------------------

static void prvSetupHardware(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   "   Webserver Demo   "
#define MESSAGE4   "                    "

/* Private macro -------------------------------------------------------------*/
#define IPV4_ADDR(n1,n2,n3,n4) ((u32_t)(n1|(n2<<8)|(n3<<16)|(n4<<24)))

/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

/* Private function prototypes -----------------------------------------------*/
void LCD_LED_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
	/*!< At this stage the microcontroller clock setting is already configured to
	 168 MHz, this is done through SystemInit() function which is called from
	 startup file (startup_stm32f4xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f4xx.c file
	 */
	prvSetupHardware();

	RCC_GetClocksFreq(&RCC_Clocks);

	adc_inicializar();

#ifdef SERIAL_DEBUG
	DebugComPort_Init();
	printf("STM32DISCOVERY is booting...\r\n");
#endif

	/*Initialize LCD and Leds */
	LCD_LED_Init();

	/*Initialize User push button */
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

	/* configure ethernet (GPIOs, clocks, MAC, DMA) */
	ETH_BSP_Config();

	/* Initilaize the LwIP stack */
	LwIP_Init();

	/* Http webserver Init */
	httpd_init();

    /* udp echo server Init */
    udp_echoserver_init();
    udp_client_init();

  //--------------------------- Creacion de las tareas del FreeRTOS ---------------------------
	//xTaskCreate(BlinkRED, (char *) "BlinkyRED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(BlinkBLUE, (char *) "BlinkyBLUE", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(BlinkORANGE, (char *) "BlinkyORANGE", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Lee_DP, (char *) "Lee_DP", 2 * configMINIMAL_STACK_SIZE, NULL, 1, NULL);

//    //Inicia FreeRTOS
	vTaskStartScheduler();
	//-------------------------------------------------------------------------------------------

	for (;;);
}

/* Chequea frames recibidos */
void Lee_DP(void *pvParameters)
{
	uint8_t rx=0, but=0;
	struct ip_addr dest_addr;

/*
	if (strcmp (p->callback, "COLOR")==0)
	{

	}
*/
	while (1)
	{
		/* check if any packet received */
		if( (rx=ETH_CheckFrameReceived()) != 0 )
		{
			/* process received ethernet packet */
			LwIP_Pkt_Handle();
		}
		/* handle periodic timers for LwIP */
		LwIP_Periodic_Handle(LocalTime);

		if(!rx)
		{
			vTaskDelay(DelayDP/portTICK_RATE_MS);		//Delay de 10mseg
		//	udp_sendto(,p,&dest_addr,8000);
		}



		if( STM_EVAL_PBGetState(BUTTON_USER) == 1 )
		{
			if(but==0)
			{
				but = 1;
				dest_addr.addr = IPV4_ADDR(192,168,0,101);		//Dirección IP de mi PC para pruebas del Ciente UDP

				udp_client(&dest_addr, 8000,"HELLO WORLD!");
			}
		}else{
			but = 0;
		}
	}
}

static void prvSetupHardware(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	 http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping(0);

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	 main() was called. */
}

//--------------------------------------------------------------
// FreeRTOS-Task "BlinkRED"
//--------------------------------------------------------------
//void BlinkRED(void *pvParameters)
//{
//	while (1)
//	{
//		STM_EVAL_LEDToggle(LED5);						//Togglea led Rojo
//
//		vTaskDelay(DelayRED / portTICK_RATE_MS);		//Delay de 1seg
//	}
//}

//--------------------------------------------------------------
// FreeRTOS-Task "BlinkBLUE"
//--------------------------------------------------------------
void BlinkBLUE(void *pvParameters)
{
	while (1)
	{
		STM_EVAL_LEDToggle(LED6);						//Togglea led Azul

		vTaskDelay(DelayBLUE / portTICK_RATE_MS);		//Delay de 500mseg
	}
}

//--------------------------------------------------------------
// FreeRTOS-Task "BlinkORANGE"
//--------------------------------------------------------------
void BlinkORANGE(void *pvParameters)
{
	while (1)
	{
		STM_EVAL_LEDToggle(LED3);					//Togglea led Naranja

		vTaskDelay(DelayORANGE / portTICK_RATE_MS);		//Delay de 200mseg
	}
}

void vApplicationTickHook(void)
{
	/* Esta funcion se habilita poniendo en 1 configUSE_TICK_HOOK en
	 * el archivo FreeRTOSConfig.h.*/

}
/*-----------------------------------------------------------*/

/**
 * @brief  Inserts a delay time.
 * @param  nCount: number of 10ms periods to wait for.
 * @retval None
 */
void Delay(uint32_t nCount)
{
	/* Capture the current local time */
	timingdelay = LocalTime + nCount;

	/* wait until the desired delay finish */
	while (timingdelay > LocalTime)
	{
	}
}

/**
 * @brief  Updates the system local time
 * @param  None
 * @retval None
 */
void Time_Update(void)									//llamada desde SysTick_Handler en port.c cada 10mSeg
{
	LocalTime += SYSTEMTICK_PERIOD_MS;
	static int i = 0;
	if (i == 1000)
	{
		STM_EVAL_LEDToggle(LED4);						//Togglea led Verde cada 10 seg
		i = 0;
	}
	++i;
}

/**
 * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
 * @param  None
 * @retval None
 */
void LCD_LED_Init(void)
{
#ifdef USE_LCD
	/* Initialize the STM324xG-EVAL's LCD */
	STM324xG_LCD_Init();
#endif

	/* Initialize STM324xG-EVAL's LEDs */
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

#ifdef USE_LCD
	/* Clear the LCD */
	LCD_Clear(Black);

	/* Set the LCD Back Color */
	LCD_SetBackColor(Black);

	/* Set the LCD Text Color */
	LCD_SetTextColor(White);

	/* Display message on the LCD*/
	LCD_DisplayStringLine(Line0, (uint8_t*)MESSAGE1);
	LCD_DisplayStringLine(Line1, (uint8_t*)MESSAGE2);
	LCD_DisplayStringLine(Line2, (uint8_t*)MESSAGE3);
	LCD_DisplayStringLine(Line3, (uint8_t*)MESSAGE4);
#endif
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
