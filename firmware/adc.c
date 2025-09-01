#include "adc.h"
#include "ch32fun.h"
#include "ch32v20xhw.h"


// ADC
ADCStateT adc_state;


void init_adc_in_pin(GPIO_TypeDef *gpio, uint32_t pin) {
	gpio->CFGLR &= ~(0xf << (4 * pin)); // CNF = 00: Analog, MODE = 00: Input
}

void adc_dma_init( void ) {
	// DMA Config
    // Start DMA clock
    RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
    Delay_Us(100);
    // Setup DMA Channel 1 (adc triggered) as reading, 16-bit, linear buffer
    DMA1_Channel1->CFGR = DMA_DIR_PeripheralSRC |
						  DMA_PeripheralInc_Disable |
                          DMA_MemoryInc_Enable |
                          DMA_PeripheralDataSize_HalfWord |
                          DMA_MemoryDataSize_HalfWord |
						  DMA_Mode_Normal |
                          DMA_M2M_Disable;
    

    // No of samples to get before irq
    DMA1_Channel1->CNTR = ADC_CHANNELS;
    // Source
    DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
    // Destination
    DMA1_Channel1->MADDR = (uint32_t)(adc_state.buffer);

    // Enable IRQ and DMA channel
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    DMA1_Channel1->CFGR |= DMA_IT_TC ;

}

void adc_init( void )
{    
	// Enable IRQ and DMA channel
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    DMA1_Channel1->CFGR |= DMA_IT_TC | DMA_CFGR1_EN;

	// ADCCLK = 12 MHz (SYSCLK=72MHz, divide by 6)
    RCC->CFGR0 &= ~RCC_ADCPRE;
    RCC->CFGR0 |= RCC_ADCPRE_DIV6;
    
    // Enable GPIOA and ADC
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1;
    
	funPinMode(PA0, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA1, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA2, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA3, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA4, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA5, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA6, GPIO_CFGLR_IN_ANALOG);
	funPinMode(PA7, GPIO_CFGLR_IN_ANALOG);

	funPinMode(PB1, GPIO_CFGLR_IN_ANALOG); // ADC IN9

    // Reset the ADC to init all regs
    RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;
    
    // Set up conversions
	ADC1->RSQR1 = ((ADC_CHANNELS - 1) << 20); // number of channels converted
	ADC1->RSQR2 = (6 << 0) | (7 << 5) | (9 << 10); // inputs 6-8
	ADC1->RSQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15) | (4 << 20) | (5 << 25);	// inputs 0-5
    
    // set sampling time for channels???
    ADC1->SAMPTR2 = ADC_SampleTime_239Cycles5 | (ADC_SampleTime_239Cycles5 << 3)  | (ADC_SampleTime_239Cycles5 << 6)  | 
		    (ADC_SampleTime_239Cycles5 << 9)  | (ADC_SampleTime_239Cycles5 << 12) | (ADC_SampleTime_239Cycles5 << 15) |
			(ADC_SampleTime_239Cycles5 << 18) | (ADC_SampleTime_239Cycles5 << 21) | (ADC_SampleTime_239Cycles5 << 24); 
			
    // turn on ADC and set rule group to sw trig
    ADC1->CTLR2 |= ADC_ADON | ADC_EXTSEL | ADC_EXTTRIG ;
	ADC1->CTLR1 |= ADC_SCAN; // scan mode, measure all channels at once
    
    // Reset calibration
    ADC1->CTLR2 |= ADC_RSTCAL;
    while(ADC1->CTLR2 & ADC_RSTCAL);
    
    // Calibrate
    ADC1->CTLR2 |= ADC_CAL;
    while(ADC1->CTLR2 & ADC_CAL){}


	adc_dma_init();

	// should be ready for SW conversion now
}

volatile uint8_t adc_dma_running = 0;
void adc_poll_dma( void )
{
	// wait if necessary
	while(adc_dma_running == 1);

	
	adc_dma_running = 1;

	// If TIM1 is running (Motor PWM), wait until it resets to get consistent readings (Always sample at the same time relative to PWM)
	if(TIM1->CTLR1 & TIM_CEN) {
		volatile uint16_t cnt = TIM1->CNT;
		// wait for rollover
		while(TIM1->CNT >= cnt) {
			cnt = TIM1->CNT;
		}
	}
	
	__disable_irq();
	// Scan
    ADC1->CTLR1 |= ADC_SCAN ;
	ADC1->CTLR2 |= ADC_DMA ;
	
    // enable dma
	DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
	
	// start sw conversion (auto clears)
	ADC1->CTLR2 |= ADC_SWSTART;
	__enable_irq();
}

void adc_wait_for_data( void ) {
	while(adc_dma_running);
	return;
}

void adc_process_data() {
	adc_state.pot[adc_state.selected_ai] = adc_state.buffer[ADC_BUFFER_POT];
	adc_state.temp_ai[adc_state.selected_ai] = adc_state.buffer[ADC_BUFFER_AI_TEMP];
	uint16_t vref = adc_state.buffer[ADC_BUFFER_VREF];
	adc_state.vref = vref;

	adc_state.isns_motor[0] = (int32_t)adc_state.buffer[ADC_BUFFER_ISNS0] - (int32_t)vref;
	adc_state.isns_motor[1] = (int32_t)adc_state.buffer[ADC_BUFFER_ISNS1] - (int32_t)vref;
	adc_state.isns_motor[2] = (int32_t)adc_state.buffer[ADC_BUFFER_ISNS2] - (int32_t)vref;
	adc_state.isns_motor[3] = (int32_t)adc_state.buffer[ADC_BUFFER_ISNS3] - (int32_t)vref;

	adc_state.vsns_bat = adc_state.buffer[ADC_BUFFER_VSNS_BAT];
	adc_state.isns_bat = (int32_t)adc_state.buffer[ADC_BUFFER_ISNS_BAT] - (int32_t)vref;

}

void DMA1_Channel1_IRQHandler()
{
    if(DMA1->INTFR & DMA1_FLAG_TC1) {
        DMA1->INTFCR = DMA_CTCIF1;
        adc_dma_init();
		adc_dma_running = 0;
    }
}

void adc_ai_select(uint8_t input) {
	if(input >= 4) {
		printf("Invalid analog input selection: %d\n", input);
		return;
	}
	funDigitalWrite(AI_SEL_A, (input & 1) != 0);
	funDigitalWrite(AI_SEL_B, (input & 2) != 0);
	adc_state.selected_ai = input;
	Delay_Us(1); // let it settle for a microsecond before sampling
}
