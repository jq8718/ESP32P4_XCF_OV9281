"""Trace three median-threshold regressions and count no-code retry work.

Only builds temporary host probes. Does not change/flash product firmware.
Uses archived Gray8 samples and shared threshold formulas from
analyze_capture_binarization.py. Windows Python and WSL gcc are required.
"""
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from analyze_capture_binarization import ROOT, masks, linux

WORK = ROOT/'build/median_regression_detail_20261118'
OUTPUT = ROOT/'docs/failure_diagnosis/binarization_20261118/median_detail'
SAMPLES = ('stutter_final01', 'stutter_restored01', 'stutter_restored_fail01')


def build():
    source = ROOT/'port/components/xcr_decoder/src/xcr_dmtx.c'
    text = source.read_text(encoding='utf-8')
    prefix = '''
#include <stdio.h>
static int analysis_pass, analysis_step, analysis_radius;
static unsigned analysis_searches, analysis_samples;
#define ANALYSIS_LOG(...) do { if (getenv("XCR_ANALYSIS_LOG")) fprintf(stderr, __VA_ARGS__); } while (0)
'''
    text = text.replace('static XcrDmtxStats last_stats;', prefix+'\nstatic XcrDmtxStats last_stats;')
    # The trace helpers occur before the definitions above.
    text = text.replace('#include <stdio.h>\nstatic void trace_marker_pgm',
                        '#include <stdio.h>\nstatic int analysis_pass, analysis_step, analysis_radius;\nstatic void trace_marker_pgm')
    text = text.replace('"%s/%s", directory, name',
                        '"%s/p%02d_s%d_r%d_%s", directory, analysis_pass, analysis_step, analysis_radius, name')
    needle = '    memset(state, 0, sizeof(*state));'
    assert text.count(needle) == 1
    text = text.replace(needle, '''    analysis_pass++;
    analysis_step = step;
    analysis_radius = radius;
    analysis_searches++;
    analysis_samples += (sample_right-sample_left)*((image->height+step-1)/step);
    ANALYSIS_LOG("SEARCH pass=%d step=%d radius=%d\\n", analysis_pass, step, radius);
'''+needle)
    needle = '    trace_marker_pgm("03_marker_binary.pgm", mask, width, height, 1);'
    text = text.replace(needle, '''    const char *mask_directory = getenv("XCR_BIN_MASK_DIR");
    if (mask_directory && *mask_directory) {
        char mask_path[1024];
        if (sample_left != 0 || width != (image->width+step-1)/step) abort();
        snprintf(mask_path, sizeof(mask_path), "%s/mask_%d.gray8", mask_directory, step);
        FILE *file = fopen(mask_path, "rb");
        if (!file || fread(mask, 1, count, file) != count) abort();
        fclose(file);
    }
'''+needle)
    needle = '            int candidate_threshold = threshold;'
    text = text.replace(needle, '''            int relevant = min_x*step>580 && min_y*step>180;
            int original_x = min_x*step, original_y = min_y*step;
            if (relevant) ANALYSIS_LOG("COMP pass=%d bbox=%d,%d,%d,%d area=%d\\n", analysis_pass,
                original_x, original_y, component_width*step, component_height*step, component_area);
'''+needle)
    needle = '            if (component_width < 2 || component_height < 2 || component_width > 32 || component_height > 32)'
    text = text.replace(needle, '''            if (relevant) ANALYSIS_LOG("FILTER pass=%d from=%d,%d bbox=%d,%d,%d,%d area=%d fill=%.1f\\n", analysis_pass,
                original_x, original_y, min_x*step, min_y*step, component_width*step, component_height*step,
                component_area, 100.0*component_area/(component_width*component_height));
'''+needle)
    needle = '                candidates[candidate_count++] = (MarkerCandidate)'
    # Wrap the single conditional body to log only accepted candidates.
    text = text.replace('            if (candidate_count < (int)(sizeof(candidates) / sizeof(candidates[0])))\n'+needle,
        '            if (candidate_count < (int)(sizeof(candidates) / sizeof(candidates[0]))) {\n'
        '                if (relevant) ANALYSIS_LOG("ACCEPT pass=%d bbox=%d,%d,%d,%d area=%d\\n", analysis_pass, min_x*step, min_y*step, component_width*step, component_height*step, component_area);\n'+needle)
    needle = '                    (dark_sum / 9 + light_sum / light_count) / 2};'
    assert text.count(needle) == 1
    text = text.replace(needle, needle+'\n            }')
    needle = '            if (candidate_count < (int)(sizeof(candidates) / sizeof(candidates[0]))) {'
    text = text.replace(needle,
        '            if (candidate_count >= (int)(sizeof(candidates) / sizeof(candidates[0])))\n'
        '                ANALYSIS_LOG("DROP_CAPACITY pass=%d bbox=%d,%d,%d,%d area=%d\\n", analysis_pass, min_x*step, min_y*step, component_width*step, component_height*step, component_area);\n'+needle)
    text = text.replace('                MarkerCandidate chosen[3] = {*a, *b, *c};',
        '                ANALYSIS_LOG("SELECT pass=%d score=%d roi=%d,%d,%d,%d\\n", analysis_pass, score, direction_roi[0], direction_roi[1], direction_roi[2], direction_roi[3]);\n'
        '                MarkerCandidate chosen[3] = {*a, *b, *c};')
    # One report at process exit, independent of stage tracing.
    text += '''
void analysis_print_counters(void) {
    fprintf(stderr, "COUNTERS searches=%u grid_samples=%u\\n", analysis_searches, analysis_samples);
}
'''
    (WORK/'current.c').write_text(text, encoding='utf-8')
    assert text.count('MarkerCandidate candidates[32];') == 1
    (WORK/'capacity128.c').write_text(text.replace('MarkerCandidate candidates[32];',
        'MarkerCandidate candidates[128];'), encoding='utf-8')
    disabled = text.replace('if (!found && step == 4)', 'if (0 && !found && step == 4)')
    disabled = disabled.replace('} else if (!image->deadline_us || xcr_port_time_us() < image->deadline_us) {',
                                '} else if (0 && (!image->deadline_us || xcr_port_time_us() < image->deadline_us)) {')
    disabled = disabled.replace('if (!decoded && sample_step == 4 && !window_retry &&',
                                'if (0 && !decoded && sample_step == 4 && !window_retry &&')
    assert disabled != text
    (WORK/'no_small.c').write_text(disabled, encoding='utf-8')
    main = (ROOT/'tools/dm_stage_trace.c').read_text()
    main = main.replace('    int status = 0;',
                        '    extern void analysis_print_counters(void);\n    atexit(analysis_print_counters);\n    int status = 0;')
    (WORK/'main.c').write_text(main, encoding='utf-8')
    shell = '''#!/bin/bash
set -euo pipefail
cd "'''+linux(ROOT)+'''"
work=build/median_regression_detail_20261118
common=(-O2 -std=c11 -fno-strict-aliasing -DCONFIG_XCR_DATAMATRIX_LIBDMTX=0 -DXCR_DM_TRACE_STAGES=1 -Iport/components/xcr_decoder/include -Iport/components/xcr_decoder/vendor/include -Iport/components/xcr_decoder/vendor/quirc)
gcc "${common[@]}" -c "$work/main.c" -o "$work/main.o"
gcc "${common[@]}" -c port/components/xcr_decoder/src/xcr_port.c -o "$work/port.o"
gcc "${common[@]}" -DXCR_DATAMATRIX_P4=1 -DXCR_PORT_REMAP_ALLOC=1 -c port/components/xcr_decoder/vendor/symbologies/datamatrix.c -o "$work/core.o"
for variant in current no_small capacity128; do
    gcc "${common[@]}" -c "$work/$variant.c" -o "$work/$variant.o"
    gcc "$work/main.o" "$work/$variant.o" "$work/port.o" "$work/core.o" -lm -o "$work/$variant"
done
'''
    (WORK/'build.sh').write_text(shell, encoding='utf-8', newline='\n')
    subprocess.run(['wsl','-e','bash',linux(WORK/'build.sh')], check=True)
    return hashlib.sha256(source.read_bytes()).hexdigest()


def run_probe(name, method, directory, trace=True, binary='current', repeats=1):
    suffix = '_capacity128' if binary=='capacity128' else ''
    destination = OUTPUT/name/(method+suffix)
    destination.mkdir(parents=True, exist_ok=True)
    environment = {'XCR_DM_TWO_STAGE':'1', 'XCR_DM_TRACE_REPEATS':str(repeats)}
    if method.startswith('median'):
        environment['XCR_BIN_MASK_DIR'] = linux(directory/method)
    if trace:
        environment.update(XCR_DM_TRACE_DIR=linux(destination), XCR_ANALYSIS_LOG='1')
    args = ['wsl','-e','env']+[k+'='+v for k,v in environment.items()]
    args += [linux(WORK/binary), linux(directory/'source.gray8')]
    run = subprocess.run(args, capture_output=True, text=True, timeout=40)
    assert run.returncode in (0,1), run.stderr
    row = json.loads(run.stdout)
    (destination/'trace.log').write_text(run.stderr, encoding='utf-8')
    (destination/'result.json').write_text(json.dumps(row,indent=2),encoding='utf-8')
    counters = run.stderr.strip().splitlines()[-1]
    row['counters'] = counters
    return row


def marker_measurements(image, selected, step):
    grid = image[::step, ::step]
    mean_mask = masks(grid)['mean13']
    median_mask = masks(grid)['median13']
    med = cv2.medianBlur(grid,13)
    rows=[]
    for x,y,w,h in selected:
        # Restrict to actual black material within the selected marker box.
        gy,gx=np.mgrid[:grid.shape[0],:grid.shape[1]]
        roi=(gx*step>=x)&(gx*step<x+w)&(gy*step>=y)&(gy*step<y+h)
        black=roi & (grid<60)
        assert np.count_nonzero(black)>0
        cy,cx=(y+h//2)//step,(x+w//2)//step
        rows.append(dict(box_px=[int(x),int(y),int(w),int(h)],
            sampled_dark_points=int(black.sum()),
            mean_black_retained=int((black & (mean_mask!=0)).sum()),
            median_black_retained=int((black & (median_mask!=0)).sum()),
            center_px=[int(cx*step),int(cy*step)],center_gray=int(grid[cy,cx]),
            median_background=int(med[cy,cx]),median_threshold=float(med[cy,cx])*.75,
            median_window_dark_fraction=float((grid[max(0,cy-6):cy+7,max(0,cx-6):cx+7]<60).mean())))
    return rows


def main():
    WORK.mkdir(parents=True,exist_ok=True)
    OUTPUT.mkdir(parents=True,exist_ok=True)
    source_hash=build()
    cases=[]
    for name in SAMPLES:
        directory=WORK/'cases'/name
        directory.mkdir(parents=True,exist_ok=True)
        image=np.fromfile(ROOT/'docs/failure_diagnosis'/f'{name}.gray8',np.uint8).reshape(720,1280)
        image.tofile(directory/'source.gray8')
        for step in (6,4):
            target=directory/'median13'
            target.mkdir(exist_ok=True)
            masks(image[::step,::step])['median13'].tofile(target/f'mask_{step}.gray8')
        current=run_probe(name,'current',directory)
        median=run_probe(name,'median13',directory)
        selected_files=sorted((OUTPUT/name/'current').glob('*04_selected_markers.csv'))
        selected=np.loadtxt(selected_files[0],delimiter=',',skiprows=1,dtype=int,ndmin=2)[:,:4]
        measurements={str(step):marker_measurements(image,selected,step) for step in (6,4)}
        # Causal check: repair just true-marker boxes in an otherwise median mask.
        for step in (6,4):
            grid=image[::step,::step]
            all_masks=masks(grid)
            repaired=all_masks['median13'].copy()
            gy,gx=np.mgrid[:grid.shape[0],:grid.shape[1]]
            for x,y,w,h in selected:
                roi=(gx*step>=x-6)&(gx*step<x+w+6)&(gy*step>=y-6)&(gy*step<y+h+6)
                repaired[roi]=all_masks['mean13'][roi]
            target=directory/'median_repaired'
            target.mkdir(exist_ok=True)
            repaired.tofile(target/f'mask_{step}.gray8')
        repaired=run_probe(name,'median_repaired',directory)
        repaired_more=run_probe(name,'median_repaired',directory,binary='capacity128')
        median_more=run_probe(name,'median13',directory,binary='capacity128')
        # Additional diagnostic: restore only the known code-body rectangle
        # to the mean mask as well. This tests spurious module candidates;
        # ground-truth corners are available only for offline diagnosis.
        corners=np.array(current['corners']).reshape(4,2)
        left,top=corners.min(axis=0)-12
        right,bottom=corners.max(axis=0)+12
        for step in (6,4):
            grid=image[::step,::step]
            all_masks=masks(grid)
            repaired_mask=np.fromfile(directory/'median_repaired'/f'mask_{step}.gray8',np.uint8).reshape(grid.shape)
            gy,gx=np.mgrid[:grid.shape[0],:grid.shape[1]]
            roi=(gx*step>=left)&(gx*step<=right)&(gy*step>=top)&(gy*step<=bottom)
            repaired_mask[roi]=all_masks['mean13'][roi]
            target=directory/'median_markers_and_body_repaired'
            target.mkdir(exist_ok=True)
            repaired_mask.tofile(target/f'mask_{step}.gray8')
        markers_body=run_probe(name,'median_markers_and_body_repaired',directory,binary='capacity128')
        cases.append(dict(name=name,current=current,median=median,repaired=repaired,
                          repaired_capacity128=repaired_more,median_capacity128=median_more,
                          markers_and_body_repaired_capacity128=markers_body,
                          markers=measurements))
        render(name,image,selected,measurements)
    # Blank and covered are original negative fixtures; scene_left is a
    # code-free crop of the real capture, resized back to CSI dimensions.
    negatives=[]
    for name in ('blank','covered'):
        negatives.append((name,np.fromfile(ROOT/'build/parallel_host_cases'/f'{name}.gray8',np.uint8).reshape(720,1280)))
    capture=sorted((ROOT/'docs/failure_diagnosis').glob('20261118*.jpg'))[0]
    photo=cv2.imread(str(capture),0)
    negatives.append(('scene_left',cv2.resize(photo[:,:480],(1280,720))))
    timing=[]
    for name,image in negatives:
        directory=WORK/'negatives'/name
        directory.mkdir(parents=True,exist_ok=True)
        image.tofile(directory/'source.gray8')
        cv2.imwrite(str(directory/'source.png'),image)
        for binary in ('current','no_small'):
            trace=run_probe(name,binary,directory,binary=binary)
            # Several process runs; host timing is illustrative only.
            timings=[]
            for _ in range(5):
                trial=run_probe(name,binary,directory,trace=False,binary=binary,repeats=32)
                assert trial['decoded']==0
                timings.append(trial['elapsed_us']/32)
            timing.append(dict(name=name,variant=binary,first_frame=trace,
                host_mean_frame_us=float(np.mean(timings)),host_process_means_us=timings,
                note='Host-only timing; 32 frames/process includes periodic native finder. Not P4 milliseconds.'))
    report=dict(source_sha256=source_hash,cases=cases,no_code=timing,
        warning='Median masks are precomputed; timings cannot compare median computational cost. Marker-box repair uses oracle boxes from baseline and is diagnostic only, not deployable detection.')
    (OUTPUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    assert hashlib.sha256((ROOT/'port/components/xcr_decoder/src/xcr_dmtx.c').read_bytes()).hexdigest()==source_hash
    assert all(c['current']['payload']=='012345' and c['current']['geometry_quality']==2 for c in cases)
    assert all(c['median']['decoded']==0 and c['median_capacity128']['decoded']==0 for c in cases)
    assert all(c['markers_and_body_repaired_capacity128']['payload']=='012345' and
               c['markers_and_body_repaired_capacity128']['geometry_quality']==2 for c in cases)
    print(json.dumps(report,ensure_ascii=False,indent=2))


def render(name,image,selected,measurements):
    canvas=Image.new('RGB',(1500,1050),'#edf2f8')
    draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype(r'C:\Windows\Fonts\msyh.ttc',20)
    title=ImageFont.truetype(r'C:\Windows\Fonts\msyh.ttc',27)
    draw.text((16,10),name+'：大实心标记在中值阈值下内部变白',font=title,fill='#18334c')
    views=[('原灰度',image),('均值背景×0.75',(1-masks(image[::6,::6])['mean13'])*255),
           ('中值背景×0.75',(1-masks(image[::6,::6])['median13'])*255)]
    for i,(label,pixels) in enumerate(views):
        step=1 if i==0 else 6
        if step==1: crop=pixels[216:720,594:1278]
        else: crop=pixels[36:120,99:213]
        enlarged=Image.fromarray(crop).convert('RGB').resize((478,352),Image.Resampling.NEAREST)
        canvas.paste(enlarged,(i*500+10,90))
        subtitle='（原图局部）' if i==0 else '（6像素网格）'
        draw.text((i*500+12,55),label+subtitle,font=font,fill='#18334c')
    draw.text((16,470),'三个真实定位标记局部：左=原灰度；中=均值；右=中值；均使用相同像素范围',font=font,fill='#18334c')
    grid=image[::6,::6]
    mean=masks(grid)['mean13']; median=masks(grid)['median13']
    for j,(x,y,w,h) in enumerate(selected):
        l=max(0,x//6-2); r=min(grid.shape[1],(x+w+5)//6+2)
        t=max(0,y//6-2); b=min(grid.shape[0],(y+h+5)//6+2)
        for i,pixels in enumerate((grid,(1-mean)*255,(1-median)*255)):
            crop=Image.fromarray(pixels[t:b,l:r]).convert('RGB').resize((145,145),Image.Resampling.NEAREST)
            canvas.paste(crop,(j*500+10+i*158,515))
        row=measurements['6'][j]
        text=f"标记{j+1}：{w}×{h} px\n黑点保留：均值{row['mean_black_retained']}/{row['sampled_dark_points']}，中值{row['median_black_retained']}/{row['sampled_dark_points']}\n中心灰度={row['center_gray']}，中值={row['median_background']}\n中值阈值={row['median_threshold']:.2f}\n窗口黑色比例={row['median_window_dark_fraction']:.1%}"
        draw.multiline_text((j*500+12,680),text,font=font,fill='#18334c',spacing=8)
    draw.text((16,885),'中值窗口被黑色标记占满 → 背景估计也变黑 → 阈值比黑色本身还低 → 黑色内部被判白',font=font,fill='#18334c')
    draw.text((16,930),'实验只替换标记掩膜；原灰度和后续码体二值化、纠错、角点精修保持相同。',font=font,fill='#18334c')
    canvas.save(OUTPUT/f'{name}_comparison.png')


if __name__=='__main__':
    main()
