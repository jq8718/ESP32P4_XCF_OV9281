#pragma once
#include <stdint.h>

typedef enum { FRAME_FREE, FRAME_WRITING, FRAME_PENDING, FRAME_DECODING } frame_state_t;
typedef struct {
    frame_state_t states[3];
    uint32_t replaced;
} latest_frame_pool_t;

static inline int latest_frame_reserve(latest_frame_pool_t *pool)
{
    for (int index = 0; index < 3; index++) {
        if (pool->states[index] == FRAME_FREE) {
            pool->states[index] = FRAME_WRITING;
            return index;
        }
    }
    return -1;
}

static inline void latest_frame_publish(latest_frame_pool_t *pool, int slot)
{
    for (int index = 0; index < 3; index++) {
        if (pool->states[index] == FRAME_PENDING) {
            pool->states[index] = FRAME_FREE;
            pool->replaced++;
        }
    }
    pool->states[slot] = FRAME_PENDING;
}

static inline int latest_frame_take(latest_frame_pool_t *pool)
{
    for (int index = 0; index < 3; index++) {
        if (pool->states[index] == FRAME_PENDING) {
            pool->states[index] = FRAME_DECODING;
            return index;
        }
    }
    return -1;
}

static inline void latest_frame_release(latest_frame_pool_t *pool, int slot)
{
    pool->states[slot] = FRAME_FREE;
}

static inline int frame_is_fresh(uint64_t capture_us, uint64_t now_us, uint64_t deadline_us)
{
    return capture_us > 0 && capture_us <= now_us && now_us < deadline_us;
}
