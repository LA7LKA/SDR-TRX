#include "ipc_mailbox.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*
 * SRAM4 (D3 domain, 0x38000000, 64 KB) is reachable by both cores and
 * isn't used by anything else in either linker script (neither script
 * even defines a region for it), so the two mailbox structs sit at fixed
 * offsets there via pointer casts rather than needing a shared linker
 * section - simpler, and nothing else can collide with it.
 */
#define IPC_CM4_TO_CM7_ADDR  (D3_SRAM_BASE + 0x0000UL)
#define IPC_CM7_TO_CM4_ADDR  (D3_SRAM_BASE + 0x1000UL)   /* well clear of the first struct */

#define IPC_CM4_TO_CM7  ((ipc_cm4_to_cm7_t *)IPC_CM4_TO_CM7_ADDR)
#define IPC_CM7_TO_CM4  ((ipc_cm7_to_cm4_t *)IPC_CM7_TO_CM4_ADDR)

void ipc_mailbox_init(void)
{
#ifdef CORE_CM7
    /*
     * Mark SRAM4 as Device memory (non-cacheable, non-bufferable, shareable)
     * so CM7's D-cache (once enabled - see the porting notes; not yet as of
     * this port) never holds a stale or torn view of what CM4 last wrote.
     * Region parameters match ST's own reference for exactly this dual-core
     * shared-memory case (STM32CubeH7 Applications/OpenAMP/OpenAMP_PingPong,
     * CM7/Src/main_cm7.c's MPU_Config()) rather than guessed values.
     */
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = D3_SRAM_BASE;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER1;   /* Number0 left free for other use */
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;   /* data only */

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif
    /* CM4 has no data cache - nothing to configure there. */

    /* First core to run this clears both structs. Harmless if both cores
       do it (whichever runs last wins, and both write all-zero/seq=0,
       which is a valid "no command/status yet" state either way) - avoids
       needing a boot-order dependency here on top of the existing
       HSEM-based CM7/CM4 boot sync CubeMX already generated. */
    memset((void *)IPC_CM4_TO_CM7, 0, sizeof(*IPC_CM4_TO_CM7));
    memset((void *)IPC_CM7_TO_CM4, 0, sizeof(*IPC_CM7_TO_CM4));
}

/*
 * Seqlock write: bump to odd (write in progress), copy every field except
 * seq itself, then bump to even (stable) - readers use the same seq to
 * detect and retry a write that was in progress mid-read. Correct because
 * each struct has exactly one writer core; if both cores could write the
 * same struct this would need a real lock (HSEM), not just a seqlock.
 */
static void seqlock_write(volatile uint32_t *seq, void *dst, const void *src, size_t n)
{
    *seq = *seq + 1;                 /* now odd */
    __DMB();
    memcpy(dst, src, n);
    __DMB();
    *seq = *seq + 1;                 /* now even */
}

static void seqlock_read(volatile uint32_t *seq, void *dst, const void *src, size_t n)
{
    uint32_t s0, s1;
    do
    {
        s0 = *seq;
        __DMB();
        memcpy(dst, src, n);
        __DMB();
        s1 = *seq;
    } while ((s0 & 1U) || (s0 != s1));   /* retry if a write was/became in progress */
}

void ipc_cm4_to_cm7_write(const ipc_cm4_to_cm7_t *cmd)
{
    seqlock_write(&IPC_CM4_TO_CM7->seq,
                  (void *)&IPC_CM4_TO_CM7->mode, &cmd->mode,
                  sizeof(*cmd) - sizeof(cmd->seq));
}

void ipc_cm4_to_cm7_read(ipc_cm4_to_cm7_t *out)
{
    seqlock_read(&IPC_CM4_TO_CM7->seq,
                &out->mode, (const void *)&IPC_CM4_TO_CM7->mode,
                sizeof(*out) - sizeof(out->seq));
}

void ipc_cm7_to_cm4_write(const ipc_cm7_to_cm4_t *status)
{
    seqlock_write(&IPC_CM7_TO_CM4->seq,
                  (void *)&IPC_CM7_TO_CM4->rx_rms, &status->rx_rms,
                  sizeof(*status) - sizeof(status->seq));
}

void ipc_cm7_to_cm4_read(ipc_cm7_to_cm4_t *out)
{
    seqlock_read(&IPC_CM7_TO_CM4->seq,
                &out->rx_rms, (const void *)&IPC_CM7_TO_CM4->rx_rms,
                sizeof(*out) - sizeof(out->seq));
}
