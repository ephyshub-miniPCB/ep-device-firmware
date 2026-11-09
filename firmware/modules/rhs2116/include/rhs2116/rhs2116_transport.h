#ifndef RHS2116_TRANSPORT_H_
#define RHS2116_TRANSPORT_H_

#include <stdint.h>

/*
 * Low level SPI transport for the RHS2116, owning the SPIM, a GPIOTE channel (CS),
 * TIMER2 (word pacing), TIMER3 (word counter) and PPI channels.
 *
 * Idle mode: rhs2116_transport_xfer() clocks one 32-bit word with its own CS pulse.
 *
 * Streaming mode: TIMER2 restarts every slot_ticks (16 MHz ticks); through PPI it
 * pulls CS low and two ticks later starts the SPIM, whose END event raises CS again
 * and counts the word on TIMER3. The SPIM uses EasyDMA ArrayList, so its TX/RX pointers step one
 * word per transfer through a ring laid out as
 *
 *     [ block A | block B | guard ]
 *
 * where the guard holds a copy of the first guard_words of block A. Once B has
 * been sent the pointers run on into the guard until the TIMER3 ISR rewinds them
 * into A at the same offset, so the chip sees an unbroken stream as long as the
 * rewind lands before the guard runs out. The rewind writes the pointers only in
 * the part of a slot where that is safe (measured, see rhs2116_transport.c), then
 * confirms the SPIM has switched before moving the TIMER3 compares. Until then a
 * compare at the end of the guard stays armed and stops TIMER2 through PPI, so a
 * late or lost rewind halts the stream instead of letting the DMA run off the ring
 * (EasyDMA writes are not checked by the MPU).
 *
 * Ring words are stored in chip (big-endian) byte order.
 */

struct rhs2116_ring {
    uint32_t *tx;
    uint32_t *rx;
    uint32_t block_words; /* words in each of block A and B */
    uint32_t guard_words; /* words in the guard, <= block_words */
};

/*
 * Called from the TIMER3 ISR each time a block has been completely clocked out.
 *   block:      0 for A, 1 for B
 *   guard_used: for A, the number of A's leading words that were actually clocked
 *               from the guard (their responses are in the RX guard, not RX A);
 *               always 0 for B
 *   err:        0, or -EOVERFLOW if the rewind missed the guard, or -EFAULT if a
 *               SPIM DMA pointer was off course; either way the stream has been
 *               stopped in hardware (no further callbacks follow)
 */
typedef void (*rhs2116_block_cb_t)(int block, uint32_t guard_used, int err);

struct rhs2116_transport_stats {
    uint32_t rewinds;
    uint32_t late_rewinds;   /* rewind attempts that missed a word's safe window */
    uint32_t max_guard_used; /* high-water mark of guard words used by a rewind */
    uint32_t pos_faults;     /* SPIM DMA pointer off course (outside the ring, or a rewind
                              * that didn't take) */
};

/* Configures the SPIM, CS, timers and PPI. Called once by the driver at boot. */
int rhs2116_transport_init(void);

/* Clocks out one word (idle mode only) and returns the word clocked in at the same
 * time, i.e. the response to the command sent two transfers earlier.
 * Returns -EBUSY while streaming. */
int rhs2116_transport_xfer(uint32_t tx, uint32_t *rx);

/* Starts timer-paced streaming through the ring. The ring must already hold valid
 * commands for A, B and the guard. Requests the HFXO and blocks until it runs. */
int rhs2116_transport_stream_start(const struct rhs2116_ring *ring, uint32_t slot_ticks,
                                   rhs2116_block_cb_t cb);

/* Stops streaming after the word in flight (if any) completes. */
void rhs2116_transport_stream_stop(void);

void rhs2116_transport_get_stats(struct rhs2116_transport_stats *stats);

#ifdef CONFIG_RHS2116_DMA_PROBE
/* Logs the SPIM's STARTED/END timing and which moments of writing the ArrayList
 * DMA pointers take effect cleanly, over a few paced words (idle mode only;
 * interrupts locked per run). */
void rhs2116_transport_dma_probe(uint32_t slot_ticks);
#endif

#endif /* RHS2116_TRANSPORT_H_ */
