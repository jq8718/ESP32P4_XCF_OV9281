from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
source = (root / 'port/components/p4scan_camera_capture/p4scan_camera_capture.c').read_text()
header = (root / 'port/components/p4scan_camera_capture/include/p4scan_camera_capture.h').read_text()
program = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <linux/videodev2.h>
typedef int esp_err_t;
typedef void *esp_timer_handle_t;
typedef void *i2c_master_bus_handle_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE -2
#define ESP_ERR_TIMEOUT -3
#define ESP_ERR_INVALID_RESPONSE -4
#define ESP_ERR_INVALID_SIZE -5
#define ESP_ERR_INVALID_ARG -6
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 0
#define VIDIOC_S_DQBUF_TIMEOUT 999
#define P4SCAN_LIGHT_SETTLE_DELAY_MS 5
#define P4SCAN_LIGHT_SETTLE_FRAMES 2
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_RETURN_ON_FALSE(condition, error, ...) do { if (!(condition)) return error; } while (0)
#define ESP_RETURN_ON_ERROR(call, ...) do { int result = (call); if (result) return result; } while (0)
'''
program += header[header.index('#define P4SCAN_CAMERA_DEFAULT_WIDTH'):header.index('void p4scan_camera_config_default')]
program += r'''
static int64_t now;
static int dequeues, queues, sync_result;
static unsigned bytesused = 16;
static bool fail_dequeue;
static uint8_t pixels[16];
static int64_t esp_timer_get_time(void) { return now; }
static void vTaskDelay(int ticks) { now += ticks * 1000; }
static void esp_timer_stop(void *timer) { (void)timer; }
static int p4scan_ov01c1b_set_lights(void *bus, bool aiming, bool light) {
    (void)bus; (void)aiming; (void)light; return 0;
}
static int p4scan_camera_set_capture_defaults(p4scan_camera_t *camera) { (void)camera; return 0; }
static int p4scan_camera_enable_lights(p4scan_camera_t *camera) { camera->lights_enabled = true; return 0; }
static int cache_sync_aligned(void *address, size_t length, int flags) {
    (void)address; (void)length; (void)flags; return sync_result;
}
static int ioctl(int fd, unsigned long request, void *argument) {
    (void)fd;
    struct v4l2_buffer *buffer = argument;
    if (request == VIDIOC_DQBUF) {
        if (fail_dequeue) { errno = EAGAIN; return -1; }
        dequeues++;
        now += 20000;
        buffer->index = 0;
        buffer->bytesused = bytesused;
        buffer->timestamp.tv_sec = now / 1000000;
        buffer->timestamp.tv_usec = now % 1000000;
    } else if (request == VIDIOC_QBUF) {
        queues++;
    }
    return 0;
}
'''
program += source[source.index('static esp_err_t wait_or_prepare_frame('):source.index('esp_err_t p4scan_camera_stop(')]
program += r'''
int main(void) {
    p4scan_camera_t camera = {.streaming = true, .held_buffer_index = -1,
        .width = 4, .height = 4, .stride = 4, .frame_rate = 50};
    camera.capture[0].address = pixels;
    camera.capture[0].length = sizeof(pixels);
    p4scan_camera_frame_t frame;
    assert(p4scan_camera_begin_capture(&camera) == ESP_OK);
    assert(camera.prepared_frame_valid && now >= 65000);
    int previous_dequeues = dequeues;
    uint64_t captured_at = camera.prepared_timestamp_us;
    now += 1000;
    assert(p4scan_camera_acquire(&camera, &frame, 1000) == ESP_OK);
    assert(dequeues == previous_dequeues && !camera.prepared_frame_valid);
    assert(frame.pixels == pixels && camera.held_buffer_index == 0);
    assert(frame.timestamp_us == captured_at && frame.timestamp_us < (uint64_t)now);
    assert(p4scan_camera_end_capture(&camera) == ESP_OK && !camera.lights_enabled);
    assert(p4scan_camera_release(&camera, &frame) == ESP_OK);
    assert(queues == dequeues);
    assert(p4scan_camera_begin_capture(&camera) == ESP_OK);
    assert(p4scan_camera_end_capture(&camera) == ESP_OK);
    assert(!camera.prepared_frame_valid && queues == dequeues);
    assert(p4scan_camera_begin_capture(&camera) == ESP_OK);
    sync_result = ESP_FAIL;
    assert(p4scan_camera_acquire(&camera, &frame, 1000) == ESP_FAIL);
    assert(camera.held_buffer_index == -1 && queues == dequeues);
    assert(p4scan_camera_end_capture(&camera) == ESP_OK);
    sync_result = 0;
    bytesused = 8;
    assert(p4scan_camera_begin_capture(&camera) == ESP_OK);
    assert(p4scan_camera_acquire(&camera, &frame, 1000) == ESP_ERR_INVALID_SIZE);
    assert(camera.held_buffer_index == -1 && queues == dequeues);
    assert(p4scan_camera_end_capture(&camera) == ESP_OK);
    fail_dequeue = true;
    assert(p4scan_camera_begin_capture(&camera) == ESP_ERR_TIMEOUT);
    assert(!camera.prepared_frame_valid && !camera.lights_enabled);
    puts("PASS retained frame, no extra dequeue, release, abort, cache failure, short frame, timeout");
}
'''
with tempfile.TemporaryDirectory(prefix='capture-prepared-') as directory:
    temporary = Path(directory)
    (temporary / 'test.c').write_text(program)
    subprocess.run(['cc', '-std=c11', '-O2', str(temporary / 'test.c'), '-o', str(temporary / 'test')], check=True)
    subprocess.run([str(temporary / 'test')], check=True)
