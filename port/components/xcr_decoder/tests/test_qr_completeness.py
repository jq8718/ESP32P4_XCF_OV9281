from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[4]
component = root / 'port/components/xcr_decoder'
source = (component / 'vendor/qr_quirc.c').read_text()
start = source.index('static int xcr_qr_grid_complete(')
function = source[start:source.index('static int xcr_finish_quirc(', start)]
program = '#include <assert.h>\n#include <stdio.h>\n#include <string.h>\n#include "quirc.h"\n' + function
program += r'''
int main(void) {
    struct quirc_code code;
    const int sizes[] = {21,29,57,177};
    int cases = 0;
    for (int size_index = 0; size_index < 4; size_index++) {
        for (int missing = -1; missing < 4; missing++) {
            memset(&code, 0, sizeof(code));
            code.size = sizes[size_index];
            for (int row = 0; row < code.size; row++) {
                for (int column = 0; column < code.size; column++) {
                    int quadrant = (column >= code.size/2) + 2*(row >= code.size/2);
                    int offset = row * code.size + column;
                    if (quadrant != missing && (row + column) % 2)
                        code.cell_bitmap[offset >> 3] |= 1U << (offset & 7);
                }
            }
            assert(xcr_qr_grid_complete(&code, 256, 256) == (missing < 0));
            cases++;
        }
    }
    memset(code.cell_bitmap, 255, sizeof(code.cell_bitmap));
    for (int point = 0; point < 4; point++) {
        code.corners[point].x = -1;
        code.corners[point].y = 257;
        assert(xcr_qr_grid_complete(&code, 256, 256));
        code.corners[point].x = -2;
        assert(!xcr_qr_grid_complete(&code, 256, 256));
        code.corners[point].x = 0;
        code.corners[point].y = 258;
        assert(!xcr_qr_grid_complete(&code, 256, 256));
        code.corners[point].y = 0;
        cases += 3;
    }
    printf("PASS %d completeness boundary/occlusion cases\n", cases);
}
'''
with tempfile.TemporaryDirectory(prefix='qr-completeness-') as directory:
    temporary = Path(directory)
    (temporary / 'test.c').write_text(program)
    subprocess.run(['cc', '-O2', '-fsanitize=address,undefined', '-I' + str(component / 'vendor/quirc'),
                    str(temporary / 'test.c'), '-o', str(temporary / 'test')], check=True)
    subprocess.run([str(temporary / 'test')], check=True)
