// startup_stm32l1.c - C-based startup code for STM32L1

#include <stdint.h>

/* Linker script symbols */
extern uint32_t _estack;        // Top of stack
extern uint32_t _sidata;        // Start of init values for .data (in flash)
extern uint32_t _sdata;         // Start of .data section (in RAM)
extern uint32_t _edata;         // End of .data section
extern uint32_t _sbss;          // Start of .bss section
extern uint32_t _ebss;          // End of .bss section

/* Function prototypes */
void Reset_Handler(void);
void Default_Handler(void);

/* Main program entry */
extern int main(void);

/* Optional system init */
void SystemInit(void) __attribute__((weak));
void SystemInit(void) {}

/* Cortex-M3 core handlers */
void NMI_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler")));

/* You can define more peripheral handlers as needed */

/* Interrupt vector table */
__attribute__((section(".isr_vector")))
const void *vector_table[] = {
    (void *)&_estack,          // Initial stack pointer
    Reset_Handler,             // Reset
    NMI_Handler,               // NMI
    HardFault_Handler,         // HardFault
    MemManage_Handler,         // MemManage
    BusFault_Handler,          // BusFault
    UsageFault_Handler,        // UsageFault
    0, 0, 0, 0,                // Reserved
    SVC_Handler,               // SVCall
    DebugMon_Handler,          // Debug monitor
    0,                         // Reserved
    PendSV_Handler,            // PendSV
    SysTick_Handler,           // SysTick
    // Add peripheral ISRs here (TIM2_IRQHandler, USART1_IRQHandler, etc.)
};

/* Reset handler */
void Reset_Handler(void) {
    uint32_t *src, *dst;

    // Copy .data section from flash to RAM
    src = &_sidata;
    dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    // Zero .bss section
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    // Optional system initialization (e.g., clock setup)
    SystemInit();

    // Call main program
    main();

    // If main returns, loop forever
    while (1);
}

/* Default handler for unused IRQs */
void Default_Handler(void) {
    while (1);
}
