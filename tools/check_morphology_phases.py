"""Run in WSL: broader morphology regression using a reusable scratch frame."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
WORK=ROOT/'build/marker_morphology_phases_20261118'
OUTPUT=ROOT/'docs/failure_diagnosis/binarization_20261118/background_exploration'
METHODS=('current','closing17','directional17','directional33')
PROBE=ROOT/'build/binarization_comparison_20261118/probe'
REPORT_FILENAME='phase_report.json'


def prepare(image):
    for step in (6,4):
        gray=image[::step,::step]
        values={}
        values['closing17']=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((17,17),np.uint8))
        for n in (17,33):
            h=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((1,n),np.uint8))
            v=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((n,1),np.uint8))
            values[f'directional{n}']=np.minimum(h,v)
        for method,background in values.items():
            folder=WORK/method
            folder.mkdir(exist_ok=True)
            (gray.astype(np.float32)<background.astype(np.float32)*.75).astype(np.uint8).tofile(folder/f'mask_{step}.gray8')


def main():
    WORK.mkdir(parents=True,exist_ok=True)
    paths=sorted((ROOT/'docs/failure_diagnosis').glob('*.gray8'))
    extra=sorted((ROOT/'build/parallel_host_cases').glob('*.gray8'))
    source=WORK/'source.gray8'
    rows=[]
    cases=[]
    for path in paths:
        cases.extend((path,'phase',dx,dy) for dy in range(6) for dx in range(6))
    # Additional rotations and scale changes on the known small label and
    # one near-field tilted label. Compare to baseline on each exact frame.
    for path in (next(p for p in paths if p.stem.startswith('20261118')),ROOT/'docs/failure_diagnosis/stutter_final01.gray8'):
        cases.extend((path,'rotate',angle,1.) for angle in (-15,-7,7,15))
        cases.extend((path,'scale',0,scale) for scale in (.65,.8,1.2,1.4))
    cases.extend((p,'negative' if p.stem in ('blank','covered') else 'extended',0,0) for p in extra)
    cache={}
    for index,(path,kind,a,b) in enumerate(cases):
        if str(path) not in cache: cache[str(path)]=np.fromfile(path,np.uint8).reshape(720,1280)
        image=cache[str(path)]
        if kind=='phase':
            shifted=np.full_like(image,128)
            shifted[b:,a:]=image[:720-b,:1280-a]
            image=shifted
        elif kind in ('rotate','scale'):
            matrix=cv2.getRotationMatrix2D((640,360),a,b)
            image=cv2.warpAffine(image,matrix,(1280,720),borderValue=128)
        image.tofile(source)
        prepare(image)
        for method in METHODS:
            env=dict(os.environ,XCR_DM_TWO_STAGE='1')
            env.pop('XCR_DM_TRACE_DIR',None)
            env.pop('XCR_BIN_MASK_DIR',None)
            if method!='current': env['XCR_BIN_MASK_DIR']=str(WORK/method)
            run=subprocess.run([str(PROBE),str(source)],env=env,capture_output=True,text=True,timeout=15)
            assert run.returncode in (0,1),(path,method,run.stderr)
            rows.append(dict(name=path.stem,kind=kind,a=a,b=b,method=method,result=json.loads(run.stdout)))
        if (index+1)%100==0: print(index+1,'/',len(cases),flush=True)
    baseline={(r['name'],r['kind'],r['a'],r['b']):r['result'] for r in rows if r['method']=='current'}
    good=lambda r:r['decoded']>0 and r['geometry_quality']==2
    summary=[]
    for method in METHODS:
        selected=[r for r in rows if r['method']==method]
        lost=[]; recovered=[]
        for r in selected:
            key=(r['name'],r['kind'],r['a'],r['b'])
            before=baseline[key]; after=r['result']
            if good(before) and (not good(after) or after['payload']!=before['payload']): lost.append(dict(name=r['name'],kind=r['kind'],a=r['a'],b=r['b']))
            if not good(before) and good(after): recovered.append(dict(name=r['name'],kind=r['kind'],a=r['a'],b=r['b'],payload=after['payload']))
        capture=[r for r in selected if r['name'].startswith('20261118') and r['kind']=='phase']
        summary.append(dict(method=method,lost=lost,recovered=recovered,
            capture_phase_total=len(capture),capture_phase_success=sum(good(r['result']) and r['result']['payload']=='012348' for r in capture),
            negative_false_positive=[r for r in selected if r['kind']=='negative' and r['result']['decoded']>0]))
    report=dict(cases=len(cases),replays=len(rows),methods=summary,source_sha256=hashlib.sha256((ROOT/'port/components/xcr_decoder/src/xcr_dmtx.c').read_bytes()).hexdigest(),
        note='Saved/reconstructed frames and synthetic transforms. No P4 timing claim; background masks precomputed.',rows=rows)
    (OUTPUT/REPORT_FILENAME).write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps([{k:len(v) if k in ('lost','recovered','negative_false_positive') else v for k,v in m.items()} for m in summary],ensure_ascii=False,indent=2))


if __name__=='__main__': main()
