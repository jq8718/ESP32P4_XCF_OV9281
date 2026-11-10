from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
component = root / 'port/components/xcr_decoder'
relative = 'port/components/xcr_decoder/vendor/qr_quirc.c'
current = (root / relative).read_text()
reference = subprocess.check_output(['git', 'show', f'27c11da:{relative}'], cwd=root).decode()


def sampler(source):
    start = source.index('static void xcr_fill_quirc_buffer(')
    return source[start:source.index('static int xcr_adaptive_threshold(', start)]


program = r'''
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "barcode.h"
#include "quirc.h"
static int xcr_min_int(int left, int right) { return left < right ? left : right; }
'''
program += sampler(reference).replace('xcr_fill_quirc_buffer', 'reference_fill')
program += sampler(current)
program += r'''
int main(void) {
    const int dimensions[][2] = {{1024,1024}, {1023,1021}, {256,128}, {57,39}};
    const int scales[] = {2,4,8};
    int cases = 0;
    for (int dimension = 0; dimension < 4; dimension++) {
        Image image = {0};
        image.width = dimensions[dimension][0]; image.height = dimensions[dimension][1];
        image.pixels = malloc((size_t)image.width * image.height);
        for (int pixel = 0; pixel < image.width * image.height; pixel++) image.pixels[pixel] = (uint8_t)rand();
        for (int crop = 0; crop <= 4; crop += 4) {
            for (int scale_index = 0; scale_index < 3; scale_index++) {
                for (int alias = 0; alias <= 2; alias++) {
                    for (int mirror = 0; mirror <= 1; mirror++) {
                        int scale = scales[scale_index];
                        int width = image.width - crop * 2, height = image.height - crop * 2;
                        struct quirc *expected = quirc_new(), *actual = quirc_new();
                        assert(expected && actual);
                        assert(!quirc_resize(expected, (width + scale - 1) / scale, (height + scale - 1) / scale));
                        assert(!quirc_resize(actual, (width + scale - 1) / scale, (height + scale - 1) / scale));
                        int expected_width, expected_height, actual_width, actual_height;
                        reference_fill(expected, &image, crop, crop, width, height, scale, mirror, alias, &expected_width, &expected_height);
                        xcr_fill_quirc_buffer(actual, &image, crop, crop, width, height, scale, mirror, alias, &actual_width, &actual_height);
                        assert(expected_width == actual_width && expected_height == actual_height);
                        assert(!memcmp(quirc_begin(expected, NULL, NULL), quirc_begin(actual, NULL, NULL), (size_t)actual_width * actual_height));
                        quirc_destroy(expected); quirc_destroy(actual); cases++;
                    }
                }
            }
        }
        free(image.pixels);
    }
    printf("PASS %d byte-identical sampling cases\n", cases);
}
'''
with tempfile.TemporaryDirectory(prefix='qr-sampling-') as directory:
    temporary = Path(directory)
    (temporary / 'test.c').write_text(program)
    subprocess.run(['cc', '-O2', '-fsanitize=address', '-I' + str(component / 'vendor/include'),
                    '-I' + str(component / 'vendor/quirc'), str(temporary / 'test.c'),
                    str(component / 'vendor/quirc/quirc.c'), str(component / 'vendor/quirc/identify.c'),
                    str(component / 'vendor/quirc/version_db.c'), '-lm', '-o', str(temporary / 'test')], check=True)
    subprocess.run([str(temporary / 'test')], check=True)
