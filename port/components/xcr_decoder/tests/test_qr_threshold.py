from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
relative = 'port/components/xcr_decoder/vendor/qr_quirc.c'
reference = subprocess.check_output(['git', 'show', f'27c11da:{relative}'], cwd=root).decode()
current = (root / relative).read_text()


def threshold(source):
    start = source.index('static int xcr_adaptive_threshold(')
    marker = 'static int xcr_qr_grid_complete(' if 'static int xcr_qr_grid_complete(' in source else 'static int xcr_finish_quirc('
    return source[start:source.index(marker, start)]


program = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct quirc { int width, height; uint8_t *pixels; };
static uint8_t *quirc_begin(struct quirc *decoder, int *width, int *height) {
    *width = decoder->width; *height = decoder->height; return decoder->pixels;
}
static int xcr_min_int(int left, int right) { return left < right ? left : right; }
static int xcr_max_int(int left, int right) { return left > right ? left : right; }
'''
program += threshold(reference).replace('xcr_adaptive_threshold', 'reference_threshold')
program += threshold(current)
program += r'''
int main(void) {
    const int dimensions[][2] = {{1,1}, {2,7}, {31,19}, {127,129}, {256,256}, {512,512}};
    const int radii[] = {0,1,12,20,80};
    uint32_t random_state = 12345;
    int cases = 0;
    for (int dimension = 0; dimension < 6; dimension++) {
        size_t size = (size_t)dimensions[dimension][0] * dimensions[dimension][1];
        struct quirc expected = {dimensions[dimension][0], dimensions[dimension][1], malloc(size)};
        struct quirc actual = {expected.width, expected.height, malloc(size)};
        assert(expected.pixels && actual.pixels);
        for (int radius_index = 0; radius_index < 5; radius_index++) {
            for (int pattern = 0; pattern < 5; pattern++) {
                for (size_t offset = 0; offset < size; offset++) {
                    random_state = random_state * 1664525U + 1013904223U;
                    uint8_t value = pattern == 0 ? 0 : pattern == 1 ? 255 :
                        pattern == 2 ? offset % 256 : pattern == 3 ? 63 + random_state % 5 : random_state >> 24;
                    expected.pixels[offset] = actual.pixels[offset] = value;
                }
                uint32_t bias = pattern == 0 ? 0 : pattern == 1 ? 255 : pattern == 2 ? 8 : 3;
                assert(reference_threshold(&expected, radii[radius_index], bias));
                assert(xcr_adaptive_threshold(&actual, radii[radius_index], bias));
                assert(!memcmp(expected.pixels, actual.pixels, size));
                cases++;
            }
        }
        free(expected.pixels); free(actual.pixels);
    }
    printf("PASS %d byte-identical threshold cases\n", cases);
}
'''
with tempfile.TemporaryDirectory(prefix='qr-threshold-') as directory:
    temporary = Path(directory)
    (temporary / 'test.c').write_text(program)
    subprocess.run(['cc', '-O2', '-fsanitize=address,undefined', str(temporary / 'test.c'),
                    '-o', str(temporary / 'test')], check=True)
    subprocess.run([str(temporary / 'test')], check=True)
