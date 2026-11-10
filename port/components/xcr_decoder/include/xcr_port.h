#ifndef XCR_PORT_H
#define XCR_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t deadline_us;
    volatile bool *cancel_flag;
} XcrDeadline;

typedef enum {
    XCR_MEM_FAST = 0,
    XCR_MEM_LARGE,
    XCR_MEM_DMA,
} XcrMemClass;

typedef enum {
    XCR_LOG_LEVEL_ERROR = 0,
    XCR_LOG_LEVEL_WARN,
    XCR_LOG_LEVEL_INFO,
    XCR_LOG_LEVEL_DEBUG,
} XcrLogLevel;

typedef struct {
    size_t current_bytes;
    size_t peak_bytes;
    uint32_t allocation_count;
    uint32_t free_count;
} XcrMemoryStats;

uint64_t xcr_port_time_us(void);
bool xcr_deadline_expired(const XcrDeadline *deadline);

void *xcr_alloc(size_t size, XcrMemClass mem_class);
void *xcr_calloc(size_t count, size_t size, XcrMemClass mem_class);
void xcr_free(void *ptr);
void xcr_memory_stats_get(XcrMemoryStats *out);
void xcr_memory_stats_reset(void);
void xcr_memory_oom_clear(void);
bool xcr_memory_oom_seen(void);

const char *xcr_port_getenv(const char *name);
void xcr_port_log(XcrLogLevel level, const char *tag, const char *format, ...);

#define XCR_LOG_ERROR(tag, ...) xcr_port_log(XCR_LOG_LEVEL_ERROR, tag, __VA_ARGS__)
#define XCR_LOG_WARN(tag, ...)  xcr_port_log(XCR_LOG_LEVEL_WARN, tag, __VA_ARGS__)
#define XCR_LOG_INFO(tag, ...)  xcr_port_log(XCR_LOG_LEVEL_INFO, tag, __VA_ARGS__)
#define XCR_LOG_DEBUG(tag, ...) xcr_port_log(XCR_LOG_LEVEL_DEBUG, tag, __VA_ARGS__)

#if defined(XCR_PORT_REMAP_ALLOC) && !defined(XCR_PORT_IMPLEMENTATION)
#define malloc(size) xcr_alloc((size), XCR_MEM_FAST)
#define calloc(count, size) xcr_calloc((count), (size), XCR_MEM_FAST)
#define free(ptr) xcr_free((ptr))
#endif

#ifdef __cplusplus
}
#endif

#endif
