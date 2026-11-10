from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
source = root / 'port/standalone_idf_test/main'
with tempfile.TemporaryDirectory(prefix='scan-log-') as directory:
    temporary = Path(directory)
    (temporary / 'freertos').mkdir()
    (temporary / 'esp_err.h').write_text('typedef int esp_err_t;\n#define ESP_OK 0\n#define ESP_ERR_NO_MEM 1\n')
    (temporary / 'esp_heap_caps.h').write_text('#include <stdlib.h>\n#define MALLOC_CAP_SPIRAM 1\n#define MALLOC_CAP_8BIT 2\n#define heap_caps_malloc(size, flags) malloc(size)\n')
    (temporary / 'esp_log.h').write_text('#include <stdarg.h>\ntypedef int (*vprintf_like_t)(const char *, va_list);\nvprintf_like_t esp_log_set_vprintf(vprintf_like_t callback);\nvoid mock_log(const char *, ...);\n#define ESP_LOGW(tag, format, ...) mock_log(format, __VA_ARGS__)\n')
    (temporary / 'freertos/FreeRTOS.h').write_text('typedef int portMUX_TYPE;\n#define portMUX_INITIALIZER_UNLOCKED 0\n#define portENTER_CRITICAL(lock) ((void)lock)\n#define portEXIT_CRITICAL(lock) ((void)lock)\n')
    (temporary / 'test.c').write_text(r'''
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "scan_log_buffer.h"
static char output[100000];
static size_t used;
static int sink(const char *format, va_list arguments) {
    int length = vsnprintf(output + used, sizeof(output) - used, format, arguments);
    assert(length >= 0 && used + length < sizeof(output));
    used += length;
    return length;
}
static vprintf_like_t writer = sink;
vprintf_like_t esp_log_set_vprintf(vprintf_like_t callback) {
    vprintf_like_t previous = writer;
    writer = callback;
    return previous;
}
void mock_log(const char *format, ...) {
    va_list arguments;
    va_start(arguments, format);
    writer(format, arguments);
    va_end(arguments);
}
int main(void) {
    assert(scan_log_init() == 0);
    scan_log_begin();
    mock_log("exposure=%d\n", 1549);
    mock_log("light=off\n");
    assert(used == 0);
    scan_log_end();
    mock_log("result=OK\n");
    scan_log_flush();
    assert(!strcmp(output, "result=OK\nexposure=1549\nlight=off\n"));
    used = 0;
    output[0] = 0;
    scan_log_begin();
    for (int index = 0; index < 4000; index++) mock_log("line=%08d\n", index);
    assert(used == 0);
    scan_log_flush();
    assert(strstr(output, "deferred log overflow: dropped="));
    used = 0;
    output[0] = 0;
    scan_log_begin();
    mock_log("error=I2C\n");
    scan_log_flush();
    assert(!strcmp(output, "error=I2C\n"));
    puts("PASS deferred ordering, overflow reporting, error flush, reuse");
}
''')
    executable = temporary / 'test'
    subprocess.run(['cc', '-std=c11', '-O2', '-I' + str(temporary), '-I' + str(source), str(temporary / 'test.c'), str(source / 'scan_log_buffer.c'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
