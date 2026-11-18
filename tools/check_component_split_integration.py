"""WSL regression of the actual integrated C code, with ASan/UBSan checks."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'build/component_split_integration_20261118'
OUT=ROOT/'docs/failure_diagnosis/component_split_20261118'


def compile_probes():
    common=['gcc','-O2','-std=c11','-fno-strict-aliasing','-DCONFIG_XCR_DATAMATRIX_LIBDMTX=0','-DXCR_DM_TRACE_STAGES=1',
        '-Iport/components/xcr_decoder/include','-Iport/components/xcr_decoder/vendor/include','-Iport/components/xcr_decoder/vendor/quirc']
    subprocess.run(common+['-c',str(WORK/'baseline_xcr_dmtx.c'),'-o',str(WORK/'baseline.o')],cwd=ROOT,check=True)
    objects=[ROOT/'build/dm_stage_trace'/f'trace_{name}.o' for name in ('main','port','core')]
    subprocess.run(['gcc',str(WORK/'baseline.o')]+list(map(str,objects))+['-lm','-o',str(WORK/'baseline')],check=True)
    sanitized=[]
    for name,path,extra in (
        ('main','tools/dm_stage_trace.c',[]),
        ('marker','port/components/xcr_decoder/src/xcr_dmtx.c',[]),
        ('port','port/components/xcr_decoder/src/xcr_port.c',[]),
        ('core','port/components/xcr_decoder/vendor/symbologies/datamatrix.c',['-DXCR_DATAMATRIX_P4=1','-DXCR_PORT_REMAP_ALLOC=1'])):
        obj=WORK/f'asan_{name}.o'
        subprocess.run(common+['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer']+extra+['-c',str(ROOT/path),'-o',str(obj)],cwd=ROOT,check=True)
        sanitized.append(str(obj))
    subprocess.run(['gcc','-no-pie','-fsanitize=address,undefined']+sanitized+['-lm','-o',str(WORK/'asan')],check=True)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    compile_probes()
    paths=sorted((ROOT/'docs/failure_diagnosis').glob('*.gray8'))
    extra=sorted((ROOT/'build/parallel_host_cases').glob('*.gray8'))
    cases=[(p,'phase',dx,dy) for p in paths for dy in range(6) for dx in range(6)]
    for path in (next(p for p in paths if p.stem.startswith('20261118')),ROOT/'docs/failure_diagnosis/stutter_final01.gray8'):
        cases += [(path,'rotate',angle,1.) for angle in (-15,-7,7,15)]
        cases += [(path,'scale',0,scale) for scale in (.65,.8,1.2,1.4)]
    cases += [(p,'negative' if p.stem in ('blank','covered') else 'extended',0,0) for p in extra]
    env=dict(os.environ,XCR_DM_TWO_STAGE='1')
    for key in ('XCR_DM_TRACE_DIR','XCR_BIN_MASK_DIR','XCR_DM_MARKER_STEP','XCR_DM_TRACE_REPEATS'):
        env.pop(key,None)
    rows=[];cache={}
    source=WORK/'regression.gray8'
    for index,(path,kind,a,b) in enumerate(cases):
        if str(path) not in cache: cache[str(path)]=np.fromfile(path,np.uint8).reshape(720,1280)
        image=cache[str(path)]
        if kind=='phase':
            shifted=np.full_like(image,128);shifted[b:,a:]=image[:720-b,:1280-a];image=shifted
        elif kind in ('rotate','scale'):
            image=cv2.warpAffine(image,cv2.getRotationMatrix2D((640,360),a,b),(1280,720),borderValue=128)
        image.tofile(source)
        results={}
        for name,binary in (('baseline',WORK/'baseline'),('integrated',ROOT/'docs/decode_stages/dm_stage_trace')):
            run=subprocess.run([str(binary),str(source)],capture_output=True,text=True,env=env,timeout=15)
            assert run.returncode in (0,1),(path,kind,a,b,name,run.stderr)
            results[name]=json.loads(run.stdout)
        rows.append(dict(name=path.stem,kind=kind,a=a,b=b,**results))
        if (index+1)%100==0: print(index+1,'/',len(cases),flush=True)
    good=lambda r:r['decoded']>0 and r['geometry_quality']==2
    lost=[r for r in rows if good(r['baseline']) and
        (not good(r['integrated']) or r['integrated']['payload']!=r['baseline']['payload'])]
    new=[r for r in rows if not good(r['baseline']) and good(r['integrated'])]
    negative=[r for r in rows if r['kind']=='negative' and r['integrated']['decoded']>0]
    captures=[r for r in rows if r['name'].startswith('20261118') and r['kind']=='phase']
    sanitizer=[]
    asan_env=dict(env,ASAN_OPTIONS='detect_leaks=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    for path in paths+extra:
        run=subprocess.run([str(WORK/'asan'),str(path)],env=asan_env,capture_output=True,text=True,timeout=25)
        assert run.returncode in (0,1) and not run.stderr,(path,run.returncode,run.stderr)
        sanitizer.append(path.stem)
    rng=np.random.default_rng(20261118)
    for index in range(12):
        if index<3: image=np.full((720,1280),[0,128,255][index],np.uint8)
        elif index<6:
            image=np.full((720,1280),200,np.uint8)
            for y in range(20,700,36):
                cv2.rectangle(image,(10,y),(1200,y+12),int(rng.integers(0,120)),-1)
        else: image=rng.integers(0,256,(720,1280),dtype=np.uint8)
        image.tofile(source)
        run=subprocess.run([str(WORK/'asan'),str(source)],env=asan_env,capture_output=True,text=True,timeout=25)
        assert run.returncode in (0,1) and not run.stderr,(index,run.returncode,run.stderr)
        sanitizer.append(f'synthetic_{index}')
    report=dict(cases=len(rows),lost=lost,recovered=new,negative_false_positive=negative,
        capture_cases=len(captures),capture_success=sum(good(r['integrated']) and r['integrated']['payload']=='012348' for r in captures),
        capture_coarse=sum(good(r['integrated']) and r['integrated']['sample_step']==6 for r in captures),
        sanitizer_cases=len(sanitizer),sanitizer_samples=sanitizer,
        source_sha256=hashlib.sha256((ROOT/'port/components/xcr_decoder/src/xcr_dmtx.c').read_bytes()).hexdigest(),rows=rows)
    (OUT/'host_regression.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:len(v) if k in ('lost','recovered','negative_false_positive') else v for k,v in report.items() if k not in ('rows','sanitizer_samples')},indent=2))
    return int(bool(lost or negative or report['capture_success']!=len(captures)))


if __name__=='__main__': raise SystemExit(main())
