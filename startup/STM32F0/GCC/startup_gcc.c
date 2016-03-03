/*******************************************************************************
@file     startup.c
@author   Rajmund Szymanski
@date     03.03.2016
@brief    STM32F0xx startup file.
          After reset the Cortex-M4 processor is in thread mode,
          priority is privileged, and the stack is set to main.
*******************************************************************************/

#ifdef  __GNUC__

#include <stm32f0xx.h>

/*******************************************************************************
 Symbols defined in linker script
*******************************************************************************/

extern unsigned  __data_init_start[];
extern unsigned       __data_start[];
extern unsigned       __data_end  [];
extern unsigned       __data_size [];
extern unsigned        __bss_start[];
extern unsigned        __bss_end  [];
extern unsigned        __bss_size [];

extern void(*__preinit_array_start[])();
extern void(*__preinit_array_end  [])();
extern void(*   __init_array_start[])();
extern void(*   __init_array_end  [])();
extern void(*   __fini_array_start[])();
extern void(*   __fini_array_end  [])();

/*******************************************************************************
 Configuration of stacks
*******************************************************************************/

#ifndef main_stack_size
#define main_stack_size   0 // <- default size of main stack
#endif
#define main_stack (((main_stack_size)+7)&(~7))

#if     main_stack_size > 0
char  __main_stack[main_stack] __attribute__ ((used, section(".main_stack")));
#endif

#ifndef proc_stack_size
#define proc_stack_size 256 // <- default size of process stack
#endif
#define proc_stack (((proc_stack_size)+7)&(~7))

#if     proc_stack_size > 0
char  __proc_stack[proc_stack] __attribute__ ((used, section(".proc_stack")));
#endif

extern  char  __initial_msp[];
extern  char  __initial_psp[];

/*******************************************************************************
 Default fault handler
*******************************************************************************/

__attribute__ ((weak, noreturn, naked)) void Fault_Handler( void )
{
	/* Go into an infinite loop */
	for (;;);
}

/*******************************************************************************
 Default exit handler
*******************************************************************************/

void _exit( int ) __attribute__ ((weak, alias("Fault_Handler")));

/*******************************************************************************
 Prototypes of external functions
*******************************************************************************/

int   main( void );

/*******************************************************************************
 Default reset procedures
*******************************************************************************/

static inline
void MemCpy( unsigned *dst_, unsigned *end_, unsigned *src_ )
{
	while (dst_ < end_) *dst_++ = *src_++;
}

static inline
void MemSet( unsigned *dst_, unsigned *end_, unsigned val_ )
{
	while (dst_ < end_) *dst_++ = val_;
}

static inline
void DataInit( void )
{
	/* Initialize the data segment */
	MemCpy(__data_start, __data_end, __data_init_start);
	/* Zero fill the bss segment */
	MemSet(__bss_start, __bss_end, 0);
}

static inline
void CallArray( void(**dst_)(), void(**end_)() )
{
	while (dst_ < end_)(*dst_++)();
}

#ifndef __NOSTARTFILES

void __libc_init_array( void );
void __libc_fini_array( void );

#else //__NOSTARTFILES

static inline
void __libc_init_array( void )
{
#ifndef __NOSTARTFILES
	CallArray(__preinit_array_start, __preinit_array_end);
	_init();
#endif
	CallArray(__init_array_start, __init_array_end);
}

static inline
void __libc_fini_array( void )
{
	CallArray(__fini_array_start, __fini_array_end);
#ifndef __NOSTARTFILES
	_fini();
#endif
}

#endif//__NOSTARTFILES

/*******************************************************************************
 Default reset handler
*******************************************************************************/

__attribute__ ((weak, noreturn, naked)) void Reset_Handler( void )
{
#if proc_stack_size > 0
	/* Initialize the process stack pointer */
	__set_PSP((unsigned)__initial_psp);
	__set_CONTROL(CONTROL_SPSEL_Msk);
#endif
#if __FPU_USED
    /* Set CP10 and CP11 Full Access */
	SCB->CPACR = 0x00F00000U;
#endif
#ifndef __NO_SYSTEM_INIT
	/* Call the system clock intitialization function */
	SystemInit();
#endif
	/* Initialize data segments */
	DataInit();
	/* Call global & static constructors */
	__libc_init_array();
	/* Call the application's entry point */
	main();
	/* Call global & static destructors */
	__libc_fini_array();
	/* Go into an infinite loop */
	_exit(0);
}

/*******************************************************************************
 Declaration of exception handlers
*******************************************************************************/

/* Core exceptions */
void NMI_Handler                  (void) __attribute__ ((weak, alias("Fault_Handler")));
void HardFault_Handler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void MemManage_Handler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void BusFault_Handler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void UsageFault_Handler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void SVC_Handler                  (void) __attribute__ ((weak, alias("Fault_Handler")));
void DebugMon_Handler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void PendSV_Handler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void SysTick_Handler              (void) __attribute__ ((weak, alias("Fault_Handler")));

/* External interrupts */
void WWDG_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void PVD_IRQHandler                (void) __attribute__ ((weak, alias("Fault_Handler")));
void RTC_IRQHandler                (void) __attribute__ ((weak, alias("Fault_Handler")));
void FLASH_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void RCC_IRQHandler                (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI0_1_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI2_3_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void EXTI4_5_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void TSC_IRQHandler                (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel1_IRQHandler      (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel2_3_IRQHandler    (void) __attribute__ ((weak, alias("Fault_Handler")));
void DMA1_Channel4_5_IRQHandler    (void) __attribute__ ((weak, alias("Fault_Handler")));
void ADC1_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM1_CC_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM2_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM3_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM6_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM7_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM14_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM15_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM16_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void TIM17_IRQHandler              (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C1_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void I2C2_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI1_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void SPI2_IRQHandler               (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART1_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART2_IRQHandler             (void) __attribute__ ((weak, alias("Fault_Handler")));
void USART3_6_IRQHandler           (void) __attribute__ ((weak, alias("Fault_Handler")));
void CEC_CAN_IRQHandler            (void) __attribute__ ((weak, alias("Fault_Handler")));
void USB_IRQHandler                (void) __attribute__ ((weak, alias("Fault_Handler")));

/*******************************************************************************
 Vector table for STM32F0xx (Cortex-M0)
*******************************************************************************/

void (* const vectors[])(void) __attribute__ ((used, section(".vectors"))) =
{
	/* Initial stack pointer */
	(void(*)(void))__initial_msp,

	/* Core exceptions */
	Reset_Handler,      /* Reset                                   */
	NMI_Handler,        /* Non-maskable interrupt                  */
	HardFault_Handler,  /* All classes of faults                   */
	MemManage_Handler,  /* Memory management                       */
	BusFault_Handler,   /* Pre-fetch fault, memory access fault    */
	UsageFault_Handler, /* Undefined instruction or illegal state  */
	0, 0, 0, 0,         /* Reserved                                */
	SVC_Handler,        /* System service call via SWI instruction */
	DebugMon_Handler,   /* Debug Monitor                           */
	0,                  /* Reserved                                */
	PendSV_Handler,     /* Pendable request for system service     */
	SysTick_Handler,    /* System tick timer                       */

#ifndef __NO_EXTERNAL_INTERRUPTS

	/* External interrupts */
	WWDG_IRQHandler,
	PVD_IRQHandler,
	RTC_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_1_IRQHandler,
	EXTI2_3_IRQHandler,
	EXTI4_5_IRQHandler,
	TSC_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_3_IRQHandler,
	DMA1_Channel4_5_IRQHandler,
	ADC1_IRQHandler,
	TIM1_BRK_UP_TRG_COM_IRQHandler,
	TIM1_CC_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM6_IRQHandler,
	TIM7_IRQHandler,
	TIM14_IRQHandler,
	TIM15_IRQHandler,
	TIM16_IRQHandler,
	TIM17_IRQHandler,
	I2C1_IRQHandler,
	I2C2_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
#if defined(USB_IRQn)||defined(CEC_CAN_IRQn)||defined(USART3_6_IRQn)||defined(USART2_IRQn)||defined(USART1_IRQn)
	USART1_IRQHandler,
#endif
#if defined(USB_IRQn)||defined(CEC_CAN_IRQn)||defined(USART3_6_IRQn)||defined(USART2_IRQn)
	USART2_IRQHandler,
#endif
#if defined(USB_IRQn)||defined(CEC_CAN_IRQn)||defined(USART3_6_IRQn)
	USART3_6_IRQHandler,
#endif
#if defined(USB_IRQn)||defined(CEC_CAN_IRQn)
	CEC_CAN_IRQHandler,
#endif
#if defined(USB_IRQn)
	USB_IRQHandler,
#endif

#endif//__NO_EXTERNAL_INTERRUPTS
};

/******************************************************************************/

#endif//__GNUC__
