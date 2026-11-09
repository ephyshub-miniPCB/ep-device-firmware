#ifndef RHS2116_PRIV_H_
#define RHS2116_PRIV_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#include <rhs2116/rhs2116.h>

// per-instance driver state, shared by rhs2116.c and rhs2116_ctrl.c
struct rhs2116_data {
    struct k_mutex bus_lock;
    enum rhs2116_stim_step step;
    int8_t neg_trim[RHS2116_NUM_CHANNELS];
    int8_t pos_trim[RHS2116_NUM_CHANNELS];
};

#endif /* RHS2116_PRIV_H_ */
