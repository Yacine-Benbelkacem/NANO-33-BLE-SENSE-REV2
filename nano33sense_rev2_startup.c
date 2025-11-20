#include <stdint.h>

/**Build:
main.o :  arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 main.c -o main.o
nano33sense_rev2_startup.o : arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -std=gnu11 nano33sense_rev2_startup.c -o nano33sense_rev2_startup.o
stm32_blink.elf : arm-none-eabi-gcc -nostdlib -T nano33sense_rev2.ld *.o -o nano33_blink.elf -Wl,-Map=nano33_blink.map

**/

/*Symbols defined in the linker script */

extern uint32_t _estack;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/* Function prototypes */

void Reset_Handler(void);
int main(void);

/* Exception and Interrupt Handlers */
void NMI_Handler					(void)__attribute__((weak,alias("Default_Handler")));
void HardFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void MemManage_Handler(void)__attribute__((weak,alias("Default_Handler")));
void BusFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void UsageFault_Handler(void)__attribute__((weak,alias("Default_Handler")));
void SVC_Handler(void)__attribute__((weak,alias("Default_Handler")));
void DebugMon_Handler(void)__attribute__((weak,alias("Default_Handler")));
void PendSV_Handler(void)__attribute__((weak,alias("Default_Handler")));
void SysTick_Handler(void)__attribute__((weak,alias("Default_Handler")));
void POWER_CLOCK_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void RADIO_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void UARTE0_UART0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void NFCT_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void GPIOTE_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SAADC_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER2_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void RTC0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TEMP_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void RNG_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void ECB_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void CCM_AAR_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void WDT_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void RTC1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void QDEC_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void COMP_LPCOMP_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI0_EGU0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI1_EGU1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI2_EGU2_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI3_EGU3_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI4_EGU4_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SWI5_EGU5_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER3_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void TIMER4_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void PWM0_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void PDM_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void MWU_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void PWM1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void PWM2_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SPIM2_SPIS2_SPI2_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void RTC2_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void I2S_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void FPU_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void USBD_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void UARTE1_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void QSPI_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void CRYPTOCELL_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void PWM3_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
void SPIM3_IRQHandler(void)__attribute__((weak,alias("Default_Handler")));
    

/* Vector Table */

uint32_t vector_tbl[] __attribute__((section(".isr_vector_tbl")))={
    (uint32_t)&   _estack,
    (uint32_t)&   Reset_Handler,
    (uint32_t)&   NMI_Handler,
    (uint32_t)&   HardFault_Handler,
    (uint32_t)&   MemManage_Handler,
    (uint32_t)&   BusFault_Handler,
    (uint32_t)&   UsageFault_Handler,
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    (uint32_t)&   SVC_Handler,
    (uint32_t)&   DebugMon_Handler,
    0,                           // Reserved
    (uint32_t)&   PendSV_Handler,
    (uint32_t)&   SysTick_Handler,
    (uint32_t)&   POWER_CLOCK_IRQHandler,
    (uint32_t)&   RADIO_IRQHandler,
    (uint32_t)&   UARTE0_UART0_IRQHandler,
    (uint32_t)&   SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,
    (uint32_t)&   SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,
    (uint32_t)&   NFCT_IRQHandler,
    (uint32_t)&   GPIOTE_IRQHandler,
    (uint32_t)&   SAADC_IRQHandler,
    (uint32_t)&   TIMER0_IRQHandler,
    (uint32_t)&   TIMER1_IRQHandler,
    (uint32_t)&   TIMER2_IRQHandler,
    (uint32_t)&   RTC0_IRQHandler,
    (uint32_t)&   TEMP_IRQHandler,
    (uint32_t)&   RNG_IRQHandler,
    (uint32_t)&   ECB_IRQHandler,
    (uint32_t)&   CCM_AAR_IRQHandler,
    (uint32_t)&   WDT_IRQHandler,
    (uint32_t)&   RTC1_IRQHandler,
    (uint32_t)&   QDEC_IRQHandler,
    (uint32_t)&   COMP_LPCOMP_IRQHandler,
    (uint32_t)&   SWI0_EGU0_IRQHandler,
    (uint32_t)&   SWI1_EGU1_IRQHandler,
    (uint32_t)&   SWI2_EGU2_IRQHandler,
    (uint32_t)&   SWI3_EGU3_IRQHandler,
    (uint32_t)&   SWI4_EGU4_IRQHandler,
    (uint32_t)&   SWI5_EGU5_IRQHandler,
    (uint32_t)&   TIMER3_IRQHandler,
    (uint32_t)&   TIMER4_IRQHandler,
    (uint32_t)&   PWM0_IRQHandler,
    (uint32_t)&   PDM_IRQHandler,
    0,                           // Reserved
    0,                          // Reserved
    (uint32_t)&   MWU_IRQHandler,
    (uint32_t)&   PWM1_IRQHandler,
    (uint32_t)&   PWM2_IRQHandler,
    (uint32_t)&   SPIM2_SPIS2_SPI2_IRQHandler,
    (uint32_t)&   RTC2_IRQHandler,
    (uint32_t)&   I2S_IRQHandler,
    (uint32_t)&   FPU_IRQHandler,
    (uint32_t)&   USBD_IRQHandler,
    (uint32_t)&   UARTE1_IRQHandler,
    (uint32_t)&   QSPI_IRQHandler,
    (uint32_t)&   CRYPTOCELL_IRQHandler,
    0,                           // Reserved
    0,                           // Reserved
    (uint32_t)&   PWM3_IRQHandler,
    0,                           // Reserved
    (uint32_t)&   SPIM3_IRQHandler,
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0,                           // Reserved
    0                           // Reserved
};

/* Default handler that enters an infinite loop */

void Default_Handler(void)
{
	while(1)
	{
		
	}
}




/* Reset Handler */
void Reset_Handler(void)
{
	// Calculate the sizes of the .data and .bss sections
	uint32_t data_mem_size =  (uint32_t)&_edata - (uint32_t)&_sdata;
	uint32_t bss_mem_size  =   (uint32_t)&_ebss - (uint32_t)&_sbss;

	/* Convert byte sizes to 32-bit word counts to match uint32_t* copies (4 == sizeof(uint32_t)) */
    data_mem_size /= 4;
    bss_mem_size  /= 4;
	
	// Initialize pointers to the source and destination of the .data section
	uint32_t *p_src_mem =  (uint32_t *)&_etext;
	uint32_t *p_dest_mem = (uint32_t *)&_sdata;
	
	/*Copy .data section from FLASH to SRAM*/
	for(uint32_t i = 0; i < data_mem_size; i++  )
	{
		
		 *p_dest_mem++ = *p_src_mem++;
	}
	
	// Initialize the .bss section to zero in SRAM
	p_dest_mem =  (uint32_t *)&_sbss;
	
	for(uint32_t i = 0; i < bss_mem_size; i++)
	{
		 /*Set bss section to zero*/  
		*p_dest_mem++ = 0;
	}
	
	    // Call the application's main function.

	main();
}




