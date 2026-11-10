import subprocess
import tempfile
from pathlib import Path


root = Path(__file__).resolve().parents[4]
source = (root / 'port/standalone_idf_test/main/camera_main.c').read_text()


def function(name):
    start = source.index('static void ' + name + '(')
    opening = source.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


start = source.index('        bool held_repeat = false;')
end = source.index('        p4scan_feedback_set_state(P4SCAN_FEEDBACK_SCANNING);', start)
accept = source[start:end]
configuration = '\n'.join(line for line in source.splitlines() if line.startswith((
    '#define SCAN_BUTTON_DEBOUNCE_MS ', '#define SCAN_BUTTON_RELEASE_MS ', '#define SCAN_BUTTON_POLL_MS ')))
program = configuration + '\n' + r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ucontext.h>
#define CONFIG_P4SCAN_TRIGGER_GPIO 35
#define GPIO_MODE_INPUT 1
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
#define pdMS_TO_TICKS(value) (value)
#define pdTRUE 1
#define ESP_OK 0
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
typedef int gpio_num_t;
typedef int esp_err_t;
typedef unsigned TickType_t;
typedef struct { uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type; } gpio_config_t;
static ucontext_t scheduler, tasks[2];
static char stacks[2][65536];
static unsigned now_ms, wake_at[2];
static int current_task, scenario, s_scan_trigger;
static bool s_scan_busy, late_timeout;
static int s_scan_button_level = 1;
static int64_t s_scan_button_down_us;
static unsigned starts[100], scan_count;
static unsigned xTaskGetTickCount(void) { return now_ms; }
static int64_t esp_timer_get_time(void) { return (int64_t)now_ms * 1000; }
static void scan_log_begin(void) {}
static int gpio_config(const gpio_config_t *config) { (void)config; return 0; }
static int gpio_get_level(gpio_num_t gpio) {
    (void)gpio;
    if (scenario) return now_ms < 400 ? 0 : 1;
    return !((now_ms >= 300 && now_ms < 4700 &&
              !(now_ms >= 900 && now_ms < 1100) && !(now_ms >= 1200 && now_ms < 1500)) ||
             (now_ms >= 6200 && now_ms < 6300) || (now_ms >= 7500 && now_ms < 7700) ||
             (now_ms >= 8000 && now_ms < 8100));
}
static void vTaskDelay(unsigned ticks) {
    wake_at[current_task] = now_ms + ticks;
    swapcontext(&tasks[current_task], &scheduler);
}
static void vTaskDelete(void *task) { (void)task; assert(0); }
static int xSemaphoreGive(int semaphore) {
    (void)semaphore;
    if (s_scan_trigger) return 0;
    s_scan_trigger = 1;
    return 1;
}
static int xSemaphoreTake(int semaphore, unsigned ticks) {
    (void)semaphore;
    unsigned deadline = now_ms + ticks;
    while (!s_scan_trigger && now_ms < deadline) vTaskDelay(1);
    if (!s_scan_trigger) return 0;
    s_scan_trigger = 0;
    return 1;
}
'''
program += function('scan_clear_pending_triggers') + '\n' + function('scan_button_task')
program += '\nstatic void button_entry(void) { scan_button_task(NULL); }\n'
program += 'static void decode_entry(void) { for (;;) { int64_t trigger_accept_us, button_down_us;\n'
program += accept
program += r'''
    assert(scan_count < 100);
    starts[scan_count++] = now_ms;
    if (!scenario && (scan_count == 2 || (now_ms >= 7500 && !late_timeout))) {
        if (now_ms >= 7500) late_timeout = true;
        vTaskDelay(3010);
    } else vTaskDelay(150);
    scan_clear_pending_triggers();
    s_scan_busy = false;
} }
int main(void) {
    for (scenario = 0; scenario < 2; scenario++) {
        now_ms = scan_count = s_scan_trigger = 0;
        wake_at[0] = wake_at[1] = 0;
        s_scan_busy = late_timeout = false;
        s_scan_button_level = 1;
        s_scan_button_down_us = 0;
        for (int index = 0; index < 2; index++) {
            getcontext(&tasks[index]);
            tasks[index].uc_stack.ss_sp = stacks[index];
            tasks[index].uc_stack.ss_size = sizeof(stacks[index]);
            tasks[index].uc_link = &scheduler;
            makecontext(&tasks[index], index ? decode_entry : button_entry, 0);
        }
        for (now_ms = 0; now_ms < 13000; now_ms++) {
            for (current_task = 0; current_task < 2; current_task++)
                if (wake_at[current_task] <= now_ms) swapcontext(&scheduler, &tasks[current_task]);
            if (!scenario) {
                if (now_ms >= 300 && now_ms < 310) assert(s_scan_button_level == 1);
                if (now_ms == 310) assert(s_scan_button_level == 0);
                if (now_ms >= 900 && now_ms < 905) assert(s_scan_button_level == 0);
                if (now_ms == 905) assert(s_scan_button_level == 1);
                if (now_ms >= 1100 && now_ms < 1110) assert(s_scan_button_level == 1);
                if (now_ms == 1110) assert(s_scan_button_level == 0);
            }
        }
        if (!scenario) {
            assert(scan_count >= 6 && starts[0] >= 310 && starts[0] < 320);
            assert(starts[2] >= starts[1] + 3010 && starts[2] < 4700);
            unsigned short_press_count = 0, timeout_press_count = 0;
            for (unsigned index = 0; index < scan_count; index++) {
                unsigned started = starts[index];
                assert(started < 4700 || (started >= 6200 && started < 6400) ||
                       (started >= 7500 && started < 7800));
                short_press_count += started >= 6200 && started < 6400;
                timeout_press_count += started >= 7500 && started < 7800;
            }
            assert(short_press_count == 1 && timeout_press_count == 1);
        } else {
            assert(scan_count >= 2 && starts[0] >= 10 && starts[0] < 20);
            assert(starts[scan_count - 1] < 400);
        }
        printf("PASS scenario=%d scans=%u last_start_ms=%u\n", scenario, scan_count, starts[scan_count - 1]);
    }
}
'''
with tempfile.TemporaryDirectory(prefix='p4-button-test-') as directory:
    test_source = Path(directory) / 'button_test.c'
    executable = Path(directory) / 'button_test'
    test_source.write_text(program)
    subprocess.run(['cc', '-std=c11', '-O2', str(test_source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
