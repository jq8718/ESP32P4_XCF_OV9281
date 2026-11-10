from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
source = (root / 'port/components/p4scan_camera_capture/p4scan_camera_capture.c').read_text()
start = source.index('static esp_err_t p4scan_camera_read_control(')
end = source.index('static void close_capture_buffers(', start)
program = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <linux/videodev2.h>
#include <errno.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_RETURN_ON_FALSE(condition, error, ...) do { if (!(condition)) return error; } while (0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define P4SCAN_INITIAL_EXPOSURE 1549
#define P4SCAN_INITIAL_GAIN_INDEX 0
typedef struct { int fd, successful_exposure, snapshot_exposure, search_exposure; bool snapshot_valid; } p4scan_camera_t;
static int exposure = 1549, gain, writes;
static bool fail_read;
static int ioctl(int fd, unsigned long request, struct v4l2_ext_controls *controls) {
    (void)fd;
    struct v4l2_ext_control *control = controls->controls;
    if (request == VIDIOC_G_EXT_CTRLS) {
        if (fail_read) return -1;
        control->value = control->id == V4L2_CID_EXPOSURE ? exposure : gain;
    } else if (control->id == V4L2_CID_GAIN) {
        gain = control->value;
    } else {
        writes++;
        int requested = control->value;
        if (requested != 1549 && requested < exposure / 2) requested = exposure / 2;
        if (requested != 1549 && requested > exposure * 2) requested = exposure * 2;
        exposure = requested;
    }
    return 0;
}
'''
program += source[start:end]
program += r'''
int main(void) {
    p4scan_camera_t camera = {0};
    assert(p4scan_camera_set_capture_defaults(&camera) == ESP_OK && exposure == 1549);
    exposure = 1200;
    p4scan_camera_snapshot_exposure(&camera);
    exposure = 19000;
    p4scan_camera_remember_exposure(&camera);
    assert(camera.successful_exposure == 1200);
    writes = 0;
    assert(p4scan_camera_set_capture_defaults(&camera) == ESP_OK && exposure == 1200 && writes > 1 && writes <= 16);
    exposure = 2000;
    p4scan_camera_snapshot_exposure(&camera);
    assert(camera.successful_exposure == 1200);
    fail_read = true;
    p4scan_camera_snapshot_exposure(&camera);
    p4scan_camera_remember_exposure(&camera);
    assert(!camera.snapshot_valid && camera.successful_exposure == 1200);
    assert(p4scan_camera_set_capture_defaults(&camera) == ESP_FAIL);
    fail_read = false;
    exposure = 1549;
    p4scan_camera_snapshot_exposure(&camera);
    p4scan_camera_adjust_search_exposure(&camera, 250);
    assert(camera.search_exposure == 774 && camera.successful_exposure == 1200);
    assert(p4scan_camera_set_capture_defaults(&camera) == ESP_OK && exposure == 774);
    p4scan_camera_snapshot_exposure(&camera);
    p4scan_camera_adjust_search_exposure(&camera, 65);
    assert(camera.search_exposure == 774);
    p4scan_camera_adjust_search_exposure(&camera, 0);
    assert(camera.search_exposure == 1548);
    camera.snapshot_exposure = 19000;
    p4scan_camera_adjust_search_exposure(&camera, 1);
    assert(camera.search_exposure == 19832);
    camera.snapshot_exposure = 16;
    p4scan_camera_adjust_search_exposure(&camera, 255);
    assert(camera.search_exposure == 16);
    camera.snapshot_valid = false;
    p4scan_camera_adjust_search_exposure(&camera, 0);
    assert(camera.search_exposure == 16);
    puts("PASS success-only seed, pre-dark snapshot, AE slew restore, read failure");
}
'''
with tempfile.TemporaryDirectory(prefix='exposure-seed-') as directory:
    temporary = Path(directory)
    (temporary / 'test.c').write_text(program)
    subprocess.run(['cc', '-std=c11', '-O2', str(temporary / 'test.c'), '-o', str(temporary / 'test')], check=True)
    subprocess.run([str(temporary / 'test')], check=True)
