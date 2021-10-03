/**
    @file
    @brief Entrada en el pin PC1 y PC2
    @date 2020/06/01
*/

#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include "adc.h"
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>

/*****************************************************************************/
/**
    @brief Prepara el sistema ADC para leer
    @returns Nada
*/
void adc_inicializar(void) {

    GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
   
    /* Puerto C -------------------------------------------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* PC1 y PC2 para entrada analógica */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_1|GPIO_Pin_2; //Seleccionamos el pin 1 y pin2 para utilizarlo como entrada.
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure); 

    /* Activar ADC2 y ADC3 ----------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    /* ADC Common Init -------------------------------------------------------*/
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode                = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler           = ADC_Prescaler_Div4; // max 36 MHz segun datasheet
    ADC_CommonInitStructure.ADC_DMAAccessMode       = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay    = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC Init ---------------------------------------------------------------*/
    ADC_StructInit (&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution             = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode           = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode     = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge   = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign              = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion        = 1;
    ADC_Init(ADC2, &ADC_InitStructure); //Guardamos la estructura configurada para inicializar el ADC2
    ADC_Init(ADC3, &ADC_InitStructure); //Guardamos la estructura configurada para inicializar el ADC3

    /* Establecer la configuración de conversión ------------------------------*/
    ADC_InjectedSequencerLengthConfig(ADC2, 1);
    ADC_SetInjectedOffset(ADC2, ADC_InjectedChannel_1, 0);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_480Cycles);// Para el ADC2 configuramos como pin entrada el PC2

    ADC_InjectedSequencerLengthConfig(ADC3, 1);
    ADC_SetInjectedOffset(ADC3, ADC_InjectedChannel_1, 0);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);// Para el ADC3 configuramos como pin entrada el PC1

    /* Poner en marcha ADC2 y el ADC3 ----------------------------------------------------*/
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
}

/*****************************************************************************/
/**
    @brief Leer tension
    @returns
*/
// Función para muestrear la señal de entrada del ADC2, y retornar un valor digital de 12 bits.
int32_t adc2_leer_cuentas(void) {
   
    uint32_t valor_adc;

    ADC_ClearFlag(ADC2,ADC_FLAG_JEOC);      // borrar flag de fin conversion
    
    ADC_SoftwareStartInjectedConv(ADC2);    // iniciar conversion

    while (ADC_GetFlagStatus(ADC2,ADC_FLAG_JEOC) == RESET); // Espera fin de conversion

    valor_adc = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1); // obtiene Valor A-D

    return valor_adc;
}
// Función para muestrear la señal de entrada del ADC3, y retornar un valor digital de 12 bits.
int32_t adc3_leer_cuentas(void) {

    uint32_t valor_adc;

    ADC_ClearFlag(ADC3,ADC_FLAG_JEOC);      // borrar flag de fin conversion

    ADC_SoftwareStartInjectedConv(ADC3);    // iniciar conversion

    while (ADC_GetFlagStatus(ADC3,ADC_FLAG_JEOC) == RESET); // Espera fin de conversion

    valor_adc = ADC_GetInjectedConversionValue(ADC3, ADC_InjectedChannel_1); // obtiene Valor A-D

    return valor_adc;
}
/*****************************************************************************/
