"""Temporary real-C directional closing prototype and host-only measurements."""
import hashlib
import json
from pathlib import Path
import subprocess

from analyze_capture_binarization import ROOT, linux

WORK=ROOT/'build/directional_closing_c_20261118'
OUTPUT=ROOT/'docs/failure_diagnosis/binarization_20261118/background_exploration'

HELPER=r'''
/* Experimental only: monotonic deque, O(pixels) per directional pass. */
static unsigned experiment_searches, experiment_samples, experiment_extra_bytes;
static void line_extreme(const uint8_t *input, uint8_t *output, int w, int h,
                         int radius, int vertical, int maximum, int *deque)
{
    int length=vertical?h:w, lines=vertical?w:h;
    for (int line=0;line<lines;line++) {
        int head=0,tail=0,next=0;
        for (int position=0;position<length;position++) {
            int right=position+radius<length?position+radius:length-1;
            while (next<=right) {
                int value=input[vertical?next*w+line:line*w+next];
                while (tail>head) {
                    int old=deque[tail-1];
                    int old_value=input[vertical?old*w+line:line*w+old];
                    if (maximum?old_value>value:old_value<value) break;
                    tail--;
                }
                deque[tail++]=next++;
            }
            int left=position>radius?position-radius:0;
            while(head<tail&&deque[head]<left) head++;
            int winner=deque[head];
            output[vertical?position*w+line:line*w+position]=
                input[vertical?winner*w+line:line*w+winner];
        }
    }
}
static int directional_binary(uint8_t *gray,int w,int h,uint8_t *workspace,size_t queue_bytes)
{
    const char *option=getenv("XCR_DIRECTIONAL_KERNEL");
    if (!option) return 0;
    int n=atoi(option);
    if (n!=17&&n!=33) return 0;
    size_t count=(size_t)w*h;
    uint8_t *temporary=workspace,*horizontal=workspace+count;
    int extra=queue_bytes<count*3;
    uint8_t *vertical=extra?xcr_alloc(count,XCR_MEM_LARGE):workspace+2*count;
    int *deque=xcr_alloc((size_t)(w>h?w:h)*sizeof(int),XCR_MEM_FAST);
    if(!vertical||!deque) abort();
    unsigned extra_bytes=(extra?count:0)+(w>h?w:h)*sizeof(int);
    if(extra_bytes>experiment_extra_bytes) experiment_extra_bytes=extra_bytes;
    line_extreme(gray,temporary,w,h,n/2,0,1,deque);
    line_extreme(temporary,horizontal,w,h,n/2,0,0,deque);
    line_extreme(gray,temporary,w,h,n/2,1,1,deque);
    line_extreme(temporary,vertical,w,h,n/2,1,0,deque);
    for(size_t i=0;i<count;i++) {
        int background=horizontal[i]<vertical[i]?horizontal[i]:vertical[i];
        gray[i]=gray[i]*100<background*75;
    }
    if(extra) xcr_free(vertical);
    xcr_free(deque);
    return 1;
}
void experiment_report(void) {
    fprintf(stderr,"COUNTERS searches=%u grid_samples=%u peak_extra_bytes=%u\n",
        experiment_searches,experiment_samples,experiment_extra_bytes);
}
'''


def main():
    WORK.mkdir(parents=True,exist_ok=True)
    product=ROOT/'port/components/xcr_decoder/src/xcr_dmtx.c'
    source_hash=hashlib.sha256(product.read_bytes()).hexdigest()
    text=product.read_text(encoding='utf-8')
    anchor='static int begin_marker_roi_slice_step('
    text=text.replace(anchor,HELPER+'\n'+anchor)
    anchor='    const size_t count = (size_t)width * (size_t)height;'
    assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'\n    experiment_searches++; experiment_samples += count;')
    begin=text.index('    if (integral_grid) {\n        for (int y')
    end=text.index('    trace_marker_pgm("03_marker_binary.pgm"',begin)
    text=text[:begin]+'    if (!directional_binary(mask,width,height,workspace,queue_bytes)) {\n'+text[begin:end]+'    }\n'+text[end:]
    # A second radius is redundant for a fixed morphological kernel. Keep
    # the product's small-window retries intact only in the mean baseline.
    text=text.replace('if (!found && step == 4)',
        'if (!getenv("XCR_DIRECTIONAL_KERNEL") && !found && step == 4)')
    text=text.replace('} else if (!image->deadline_us || xcr_port_time_us() < image->deadline_us) {',
        '} else if (!getenv("XCR_DIRECTIONAL_KERNEL") && (!image->deadline_us || xcr_port_time_us() < image->deadline_us)) {')
    text=text.replace('if (!decoded && sample_step == 4 && !window_retry &&',
        'if (!getenv("XCR_DIRECTIONAL_KERNEL") && !decoded && sample_step == 4 && !window_retry &&')
    (WORK/'marker.c').write_text(text,encoding='utf-8')
    main=(ROOT/'tools/dm_stage_trace.c').read_text()
    main=main.replace('    int status = 0;',
        '    extern void experiment_report(void); atexit(experiment_report);\n    int status=0;')
    (WORK/'main.c').write_text(main,encoding='utf-8')
    shell='''#!/bin/bash
set -euo pipefail
cd "'''+linux(ROOT)+'''"
work=build/directional_closing_c_20261118
common=(-O2 -std=c11 -fno-strict-aliasing -DCONFIG_XCR_DATAMATRIX_LIBDMTX=0 -DXCR_DM_TRACE_STAGES=1 -Iport/components/xcr_decoder/include -Iport/components/xcr_decoder/vendor/include -Iport/components/xcr_decoder/vendor/quirc)
gcc "${common[@]}" -c "$work/main.c" -o "$work/main.o"
gcc "${common[@]}" -c "$work/marker.c" -o "$work/marker.o"
gcc "$work/main.o" "$work/marker.o" build/binarization_comparison_20261118/port.o build/binarization_comparison_20261118/core.o -lm -o "$work/probe"
'''
    (WORK/'build.sh').write_text(shell,encoding='utf-8',newline='\n')
    subprocess.run(['wsl','-e','bash',linux(WORK/'build.sh')],check=True)
    runner='''import hashlib,json,os,subprocess
from pathlib import Path
import cv2,numpy as np
work=Path(__file__).resolve().parent
root=work.parents[1]
paths=sorted((root/'docs/failure_diagnosis').glob('*.gray8'))
paths+=sorted((root/'build/parallel_host_cases').glob('*.gray8'))
rows=[];verified=0
for path in paths:
    image=np.fromfile(path,np.uint8).reshape(720,1280)
    for method,kernel in (('current',None),('directional17',17),('directional33',33)):
        env=dict(os.environ,XCR_DM_TWO_STAGE='1')
        if kernel: env['XCR_DIRECTIONAL_KERNEL']=str(kernel)
        trace=work/'trace';trace.mkdir(exist_ok=True)
        env['XCR_DM_TRACE_DIR']=str(trace)
        run=subprocess.run([str(work/'probe'),str(path)],env=env,capture_output=True,text=True)
        assert run.returncode in (0,1),run.stderr
        result=json.loads(run.stdout)
        if kernel:
            for step in (6,4):
                # Force each grid once so correctness is checked even when
                # first-pass decoding succeeds without using the dense grid.
                env['XCR_DM_MARKER_STEP']=str(step)
                check=subprocess.run([str(work/'probe'),str(path)],env=env,capture_output=True,text=True)
                assert check.returncode in (0,1),check.stderr
                observed=cv2.imread(str(trace/'03_marker_binary.pgm'),0)
                # If sparse decoding failed, the final dump is the 4px pass.
                actual_step=6 if observed.shape==(120,214) else 4
                gray=image[::actual_step,::actual_step]
                h=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((1,kernel),np.uint8))
                v=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((kernel,1),np.uint8))
                expected=np.where(gray.astype(np.int32)*100<np.minimum(h,v).astype(np.int32)*75,0,255).astype(np.uint8)
                assert np.array_equal(observed,expected),(path,method,step,np.count_nonzero(observed!=expected))
                verified+=1
            env.pop('XCR_DM_MARKER_STEP',None)
        env.pop('XCR_DM_TRACE_DIR',None)
        env['XCR_DM_TRACE_REPEATS']='32'
        trial=subprocess.run([str(work/'probe'),str(path)],env=env,capture_output=True,text=True)
        assert trial.returncode in (0,1),trial.stderr
        measured=json.loads(trial.stdout)
        rows.append(dict(name=path.stem,method=method,first_frame=result,host_mean_us=measured['elapsed_us']/32,counters=trial.stderr.strip()))
(work/'results.json').write_text(json.dumps(dict(mask_checks=verified,rows=rows),indent=2))
print('Verified masks',verified,'and measured',len(rows),'32-frame host batches')
'''
    (WORK/'run.py').write_text(runner,encoding='utf-8')
    subprocess.run(['wsl','-e','python3',linux(WORK/'run.py')],check=True)
    report=json.loads((WORK/'results.json').read_text())
    report.update(source_sha256=source_hash,note='Real C prototype includes deque morphology, allocation, search, decode and refinement; host times are not P4 timings. New method suppresses redundant identical-mask small-window retries. Extra memory reuses the existing candidate queue.')
    (OUTPUT/'c_prototype_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    assert source_hash==hashlib.sha256(product.read_bytes()).hexdigest()
    print('Product source unchanged.')


if __name__=='__main__': main()
