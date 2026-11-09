/* Minimal stand-ins for the Zephyr APIs used by rhs2116_stream.c and stim.c, so
 * they can be compiled and exercised on the host. Single threaded: nothing
 * blocks, semaphores and queues just report empty. */
#ifndef HOST_SHIM_H
#define HOST_SHIM_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BIT(n)                (1UL << (n))
#define MIN(a, b)             ((a) < (b) ? (a) : (b))
#define MAX(a, b)             ((a) > (b) ? (a) : (b))
#define CLAMP(v, lo, hi)      MIN(MAX((v), (lo)), (hi))
#define DIV_ROUND_UP(n, d)    (((n) + (d) - 1) / (d))
#define ARRAY_SIZE(a)         (sizeof(a) / sizeof((a)[0]))
#define ARG_UNUSED(x)         (void)(x)
#define POPCOUNT(x)           __builtin_popcount(x)
#define __aligned(x)          __attribute__((aligned(x)))
#define BUILD_ASSERT(...)
#define __ASSERT_NO_MSG(x)

#define LOG_MODULE_REGISTER(...)
#define LOG_MODULE_DECLARE(...)
#define LOG_ERR(...) (fprintf(stderr, "E: " __VA_ARGS__), fputc('\n', stderr))
#define LOG_WRN(...) (fprintf(stderr, "W: " __VA_ARGS__), fputc('\n', stderr))
#define LOG_INF(...) do { } while (0)
#define LOG_DBG(...) do { } while (0)

typedef long atomic_t;
typedef long atomic_val_t;
typedef void *atomic_ptr_t;
typedef void *atomic_ptr_val_t;
static inline atomic_val_t atomic_get(const atomic_t *a) { return *a; }
static inline atomic_val_t atomic_set(atomic_t *a, atomic_val_t v) { atomic_val_t o = *a; *a = v; return o; }
static inline atomic_val_t atomic_clear(atomic_t *a) { return atomic_set(a, 0); }
static inline atomic_val_t atomic_inc(atomic_t *a) { return (*a)++; }
static inline bool atomic_cas(atomic_t *a, atomic_val_t o, atomic_val_t n) {
    if (*a != o) return false;
    *a = n;
    return true;
}
static inline void *atomic_ptr_get(const atomic_ptr_t *a) { return *a; }
static inline void atomic_ptr_set(atomic_ptr_t *a, void *v) { *a = v; }

typedef struct { int unused; } k_timeout_t;
#define K_FOREVER      ((k_timeout_t){0})
#define K_NO_WAIT      ((k_timeout_t){0})
#define K_MSEC(x)      ((k_timeout_t){0})
#define K_SECONDS(x)   ((k_timeout_t){0})
#define K_USEC(x)      ((k_timeout_t){0})
#define K_PRIO_COOP(x) (-(x))

struct k_spinlock { int unused; };
typedef int k_spinlock_key_t;
static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l) { (void)l; return 0; }
static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t k) { (void)l; (void)k; }

struct k_mutex { int unused; };
#define K_MUTEX_DEFINE(name) struct k_mutex name
static inline void k_mutex_init(struct k_mutex *m) { (void)m; }
static inline int k_mutex_lock(struct k_mutex *m, k_timeout_t t) { (void)m; (void)t; return 0; }
static inline void k_mutex_unlock(struct k_mutex *m) { (void)m; }

struct k_sem { int count; int limit; };
#define K_SEM_DEFINE(name, init, lim) struct k_sem name = { (init), (lim) }
static inline void k_sem_init(struct k_sem *s, int i, int l) { s->count = i; s->limit = l; }
static inline void k_sem_give(struct k_sem *s) { if (s->count < s->limit) s->count++; }
static inline int k_sem_take(struct k_sem *s, k_timeout_t t) {
    (void)t;
    if (s->count == 0) return -EAGAIN;
    s->count--;
    return 0;
}
static inline void k_sem_reset(struct k_sem *s) { s->count = 0; }

struct k_msgq { char *buf; size_t msg_size; uint32_t depth; uint32_t head, used; };
#define K_MSGQ_DEFINE(name, size, depth_, align)                                  \
    static char name##_buf[(size) * (depth_)];                                     \
    struct k_msgq name = { name##_buf, (size), (depth_), 0, 0 }
static inline int k_msgq_put(struct k_msgq *q, const void *m, k_timeout_t t) {
    (void)t;
    if (q->used == q->depth) return -ENOMSG;
    memcpy(q->buf + ((q->head + q->used) % q->depth) * q->msg_size, m, q->msg_size);
    q->used++;
    return 0;
}
static inline int k_msgq_get(struct k_msgq *q, void *m, k_timeout_t t) {
    (void)t;
    if (q->used == 0) return -ENOMSG;
    memcpy(m, q->buf + q->head * q->msg_size, q->msg_size);
    q->head = (q->head + 1) % q->depth;
    q->used--;
    return 0;
}
static inline void k_msgq_purge(struct k_msgq *q) { q->used = 0; }

typedef void *k_tid_t;
#define K_THREAD_DEFINE(name, stack, fn, a, b, c, prio, opt, delay)               \
    const k_tid_t name = (k_tid_t)&fn
static inline k_tid_t k_current_get(void) { return NULL; }

#define SYS_INIT(fn, level, prio) int (*const fn##_sys_init)(void) = fn

struct k_work;
typedef void (*k_work_handler_t)(struct k_work *);
struct k_work { k_work_handler_t handler; int submitted; };
#define K_WORK_DEFINE(name, h) struct k_work name = { h, 0 }
static inline int k_work_submit(struct k_work *w) { w->submitted++; return 0; }
static inline void k_busy_wait(uint32_t us) { (void)us; }

struct device { const void *config; void *data; };

static inline uint32_t sys_cpu_to_be32(uint32_t v) { return __builtin_bswap32(v); }
static inline uint32_t sys_be32_to_cpu(uint32_t v) { return __builtin_bswap32(v); }

#endif
