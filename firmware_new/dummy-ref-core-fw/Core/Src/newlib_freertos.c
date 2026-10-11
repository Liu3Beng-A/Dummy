#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include <malloc.h>

/*
 * Per-task _reent protects float conversion state, but malloc's arena is global.
 * USB CDC init/deinit also calls malloc/free from OTG_FS_IRQn (priority 5), so a
 * scheduler lock or a task mutex alone would leave the allocator reentrant.
 *
 * Mask only the RTOS interrupt priorities while the allocator owns its arena.
 * This also prevents task switches and works before the scheduler starts, during
 * task cleanup, and inside an existing critical section. Higher priority ISRs
 * remain enabled and MUST NOT allocate. No formatting or I/O is done under this
 * lock. This does not make printf/scanf safe to call from interrupts.
 */
static uint32_t mallocLockDepth;
static uint32_t mallocSavedBasepri;

void __malloc_lock(struct _reent *reent)
{
    (void)reent;
    const uint32_t exception = __get_IPSR();
    if (exception != 0U)
    {
        /* Core exceptions and interrupts above the mask may not use newlib. */
        configASSERT(exception >= 16U);
        configASSERT(NVIC->IP[exception - 16U] >= configMAX_SYSCALL_INTERRUPT_PRIORITY);
    }

    const uint32_t previous = __get_BASEPRI();
    /* BASEPRI_MAX must not weaken an already stricter caller's mask. */
    __set_BASEPRI_MAX(configMAX_SYSCALL_INTERRUPT_PRIORITY);
    __DSB();
    __ISB();

    configASSERT(mallocLockDepth != UINT32_MAX);
    if (mallocLockDepth == 0U)
        mallocSavedBasepri = previous;
    ++mallocLockDepth;
}

void __malloc_unlock(struct _reent *reent)
{
    (void)reent;
    configASSERT(mallocLockDepth != 0U);
    if (--mallocLockDepth == 0U)
    {
        const uint32_t previous = mallocSavedBasepri;
        __DMB();
        __set_BASEPRI(previous);
        __ISB();
    }
}
