#ifndef ESP_PLATFORM
#define _POSIX_C_SOURCE 200809L
#endif
#define XCR_PORT_IMPLEMENTATION 1
#include "xcr_port.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdatomic.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#endif

#define XCR_ALLOC_MAGIC 0x5843524DU
#define XCR_LOG_BUFFER_SIZE 384

typedef union {
    struct {
        size_t size;
        uint32_t magic;
    } metadata;
#ifdef _MSC_VER
    long double alignment;
#else
    max_align_t alignment;
#endif
} XcrAllocationHeader;

static XcrMemoryStats g_memory_stats;
static atomic_bool g_oom_seen;
#ifdef ESP_PLATFORM
static portMUX_TYPE g_memory_stats_lock = portMUX_INITIALIZER_UNLOCKED;
#define MEMORY_STATS_LOCK() portENTER_CRITICAL(&g_memory_stats_lock)
#define MEMORY_STATS_UNLOCK() portEXIT_CRITICAL(&g_memory_stats_lock)
#else
#define MEMORY_STATS_LOCK() ((void)0)
#define MEMORY_STATS_UNLOCK() ((void)0)
#endif

uint64_t xcr_port_time_us(void)
{
#ifdef ESP_PLATFORM
    return (uint64_t)esp_timer_get_time();
#elif defined(_WIN32)
    static LARGE_INTEGER frequency;
    LARGE_INTEGER counter;
    if (frequency.QuadPart == 0) {
        QueryPerformanceFrequency(&frequency);
    }
    QueryPerformanceCounter(&counter);
    return (uint64_t)((counter.QuadPart * 1000000ULL) /
                      (uint64_t)frequency.QuadPart);
#else
    struct timespec timestamp;
    if (clock_gettime(CLOCK_MONOTONIC, &timestamp) != 0) {
        return 0;
    }
    return (uint64_t)timestamp.tv_sec * 1000000ULL +
           (uint64_t)timestamp.tv_nsec / 1000ULL;
#endif
}

bool xcr_deadline_expired(const XcrDeadline *deadline)
{
    if (!deadline) {
        return false;
    }
    if (deadline->cancel_flag && *deadline->cancel_flag) {
        return true;
    }
    return deadline->deadline_us != 0 && xcr_port_time_us() >= deadline->deadline_us;
}

static void *xcr_raw_alloc(size_t size, XcrMemClass mem_class)
{
#ifdef ESP_PLATFORM
    uint32_t caps = MALLOC_CAP_8BIT;
    void *memory;

    if (mem_class == XCR_MEM_FAST) {
        caps |= MALLOC_CAP_INTERNAL;
    } else if (mem_class == XCR_MEM_LARGE) {
        caps |= MALLOC_CAP_SPIRAM;
    } else if (mem_class == XCR_MEM_DMA) {
        caps |= MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;
    }

    memory = heap_caps_malloc(size, caps);
    if (!memory && mem_class == XCR_MEM_LARGE) {
        memory = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }
    return memory;
#else
    (void)mem_class;
    return malloc(size);
#endif
}

static void xcr_raw_free(void *ptr)
{
#ifdef ESP_PLATFORM
    heap_caps_free(ptr);
#else
    free(ptr);
#endif
}

void *xcr_alloc(size_t size, XcrMemClass mem_class)
{
    XcrAllocationHeader *header;
    size_t total_size;

    if (size == 0 || size > SIZE_MAX - sizeof(*header)) {
        return NULL;
    }
    total_size = sizeof(*header) + size;
    header = (XcrAllocationHeader *)xcr_raw_alloc(total_size, mem_class);
    if (!header) {
        g_oom_seen = true;
        return NULL;
    }
    header->metadata.size = size;
    header->metadata.magic = XCR_ALLOC_MAGIC;
    MEMORY_STATS_LOCK();
    g_memory_stats.current_bytes += size;
    if (g_memory_stats.current_bytes > g_memory_stats.peak_bytes) {
        g_memory_stats.peak_bytes = g_memory_stats.current_bytes;
    }
    g_memory_stats.allocation_count++;
    MEMORY_STATS_UNLOCK();
    return header + 1;
}

void *xcr_calloc(size_t count, size_t size, XcrMemClass mem_class)
{
    void *memory;
    size_t total_size;

    if (count == 0 || size == 0 || count > SIZE_MAX / size) {
        return NULL;
    }
    total_size = count * size;
    memory = xcr_alloc(total_size, mem_class);
    if (memory) {
        memset(memory, 0, total_size);
    }
    return memory;
}

void xcr_free(void *ptr)
{
    XcrAllocationHeader *header;

    if (!ptr) {
        return;
    }
    header = ((XcrAllocationHeader *)ptr) - 1;
    if (header->metadata.magic != XCR_ALLOC_MAGIC) {
        return;
    }
    header->metadata.magic = 0;
    MEMORY_STATS_LOCK();
    if (g_memory_stats.current_bytes >= header->metadata.size) {
        g_memory_stats.current_bytes -= header->metadata.size;
    } else {
        g_memory_stats.current_bytes = 0;
    }
    g_memory_stats.free_count++;
    MEMORY_STATS_UNLOCK();
    xcr_raw_free(header);
}

void xcr_memory_stats_get(XcrMemoryStats *out)
{
    if (out) {
        MEMORY_STATS_LOCK();
        *out = g_memory_stats;
        MEMORY_STATS_UNLOCK();
    }
}

void xcr_memory_stats_reset(void)
{
    MEMORY_STATS_LOCK();
    size_t current_bytes = g_memory_stats.current_bytes;
    memset(&g_memory_stats, 0, sizeof(g_memory_stats));
    g_memory_stats.current_bytes = current_bytes;
    g_memory_stats.peak_bytes = current_bytes;
    MEMORY_STATS_UNLOCK();
}

void xcr_memory_oom_clear(void)
{
    g_oom_seen = false;
}

bool xcr_memory_oom_seen(void)
{
    return g_oom_seen;
}

const char *xcr_port_getenv(const char *name)
{
#ifdef ESP_PLATFORM
    (void)name;
    return NULL;
#else
    return getenv(name);
#endif
}

void xcr_port_log(XcrLogLevel level, const char *tag, const char *format, ...)
{
    va_list arguments;

    if (!format) {
        return;
    }
    va_start(arguments, format);
#ifdef ESP_PLATFORM
    char buffer[XCR_LOG_BUFFER_SIZE];
    vsnprintf(buffer, sizeof(buffer), format, arguments);
    if (level == XCR_LOG_LEVEL_ERROR) {
        ESP_LOGE(tag ? tag : "xcr", "%s", buffer);
    } else if (level == XCR_LOG_LEVEL_WARN) {
        ESP_LOGW(tag ? tag : "xcr", "%s", buffer);
    } else if (level == XCR_LOG_LEVEL_INFO) {
        ESP_LOGI(tag ? tag : "xcr", "%s", buffer);
    } else {
        ESP_LOGD(tag ? tag : "xcr", "%s", buffer);
    }
#else
    static const char *levels[] = {"E", "W", "I", "D"};
    fprintf(stderr, "[%s][%s] ", levels[level], tag ? tag : "xcr");
    vfprintf(stderr, format, arguments);
    fputc('\n', stderr);
#endif
    va_end(arguments);
}
