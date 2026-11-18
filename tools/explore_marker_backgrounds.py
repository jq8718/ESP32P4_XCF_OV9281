"""Compare morphology, directional closing, tile thresholds and MSER offline.

No firmware edits/flashing. Only marker masks change in a temporary C probe.
OpenCV implementations are evidence of detection behavior, not P4 benchmarks.
"""
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np

from analyze_capture_binarization import ROOT, build_probe, linux

WORK = ROOT/'build/marker_background_exploration_20261118'
OUTPUT = ROOT/'docs/failure_diagnosis/binarization_20261118/background_exploration'
METHODS = ('current','closing9','closing17','closing25','closing41',
           'directional17','directional33','tile_otsu8',
           'mser_delta1','mser_delta3','mser_delta5','contours')


def candidates(gray):
    result={}
    g=gray.astype(np.float32)
    for n in (9,17,25,41):
        closed=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((n,n),np.uint8))
        result[f'closing{n}']=(g<closed.astype(np.float32)*.75).astype(np.uint8)
    # A paper edge is often open to one direction. Fill dark objects along
    # each axis, and use the lower of those two backgrounds to avoid letting
    # a bright boundary in just one axis dictate the threshold.
    for n in (17,33):
        h=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((1,n),np.uint8))
        v=cv2.morphologyEx(gray,cv2.MORPH_CLOSE,np.ones((n,1),np.uint8))
        result[f'directional{n}']=(g<np.minimum(h,v).astype(np.float32)*.75).astype(np.uint8)
    h,w=gray.shape
    thresholds=np.zeros(((h+7)//8,(w+7)//8),np.float32)
    for by,y in enumerate(range(0,h,8)):
        for bx,x in enumerate(range(0,w,8)):
            block=gray[y:y+8,x:x+8]
            if int(block.max())-int(block.min())<24:
                thresholds[by,bx]=float(block.min())*.5
            else:
                thresholds[by,bx]=cv2.threshold(block,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)[0]
    # Tile centers and interpolation are a simple research baseline, not a
    # copy of the ZXing hybrid binarizer's neighbor-blackpoint rules.
    local=cv2.resize(thresholds,(w,h),interpolation=cv2.INTER_LINEAR)
    result['tile_otsu8']=(g<=local).astype(np.uint8)
    for delta in (1,3,5):
        mask=np.zeros_like(gray)
        detector=cv2.MSER_create(delta,4,1024,.5,.2)
        regions,boxes=detector.detectRegions(gray)
        for points,(x,y,bw,bh) in zip(regions,boxes):
            if bw<2 or bh<2 or bw>32 or bh>32 or min(bw,bh)<max(bw,bh)*.6:
                continue
            if len(points)<bw*bh*.4: continue
            values=gray[points[:,1],points[:,0]]
            surround=gray[max(0,y-2):min(h,y+bh+2),max(0,x-2):min(w,x+bw+2)]
            if float(np.mean(values))+3>=float(np.mean(surround)): continue
            mask[points[:,1],points[:,0]]=1
        result[f'mser_delta{delta}']=mask
    mask=np.zeros_like(gray)
    edges=cv2.Canny(gray,15,40)
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        perimeter=cv2.arcLength(contour,True)
        poly=cv2.approxPolyDP(contour,perimeter*.045,True)
        if len(poly)!=4 or not cv2.isContourConvex(poly): continue
        x,y,bw,bh=cv2.boundingRect(poly)
        area=cv2.contourArea(poly)
        if bw<2 or bh<2 or max(bw,bh)>32 or min(bw,bh)<max(bw,bh)*.6: continue
        if area<bw*bh*.4: continue
        inside=np.zeros_like(gray)
        cv2.fillPoly(inside,[poly],1)
        interior=gray[inside!=0]
        surround=gray[max(0,y-2):min(h,y+bh+2),max(0,x-2):min(w,x+bw+2)]
        if len(interior) and float(interior.mean())+3<float(surround.mean()):
            cv2.fillPoly(mask,[poly],1)
    result['contours']=mask
    return result


def main():
    WORK.mkdir(parents=True,exist_ok=True)
    OUTPUT.mkdir(parents=True,exist_ok=True)
    source_hash=build_probe()
    archived=ROOT/'docs/failure_diagnosis'
    new_paths=sorted(archived.glob('20261118*.gray8'))
    assert len(new_paths)==5
    historical=[p for p in sorted(archived.glob('*.gray8')) if p not in new_paths]
    cases=[]
    for i,path in enumerate(new_paths):
        image=np.fromfile(path,np.uint8).reshape(720,1280)
        for dy in range(4):
            for dx in range(4):
                shifted=np.full_like(image,128)
                shifted[dy:,dx:]=image[:720-dy,:1280-dx]
                cases.append((f'capture_{i}_{dx}_{dy}',shifted,'capture',dx,dy))
    for path in historical:
        cases.append((path.stem,np.fromfile(path,np.uint8).reshape(720,1280),'historical',0,0))
    for path in sorted((ROOT/'build/parallel_host_cases').glob('*.gray8')):
        group='negative' if path.stem in ('blank','covered') else 'extended'
        cases.append((path.stem,np.fromfile(path,np.uint8).reshape(720,1280),group,0,0))
    # A real no-code scene, derived from the capture's code-free left area.
    image=np.fromfile(new_paths[0],np.uint8).reshape(720,1280)
    cases.append(('scene_left',cv2.resize(image[:,:480],(1280,720)),'negative',0,0))
    manifest=[]
    for index,(name,image,group,dx,dy) in enumerate(cases):
        folder=WORK/'cases'/name
        folder.mkdir(parents=True,exist_ok=True)
        image.tofile(folder/'source.gray8')
        for step in (6,4):
            for method,mask in candidates(image[::step,::step]).items():
                target=folder/method
                target.mkdir(exist_ok=True)
                mask.tofile(target/f'mask_{step}.gray8')
        manifest.append(dict(name=name,group=group,dx=dx,dy=dy,directory=linux(folder)))
        if (index+1)%20==0: print('Prepared',index+1,'/',len(cases),flush=True)
    (WORK/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
    runner='''import json, os, subprocess
from pathlib import Path
work=Path(__file__).resolve().parent
root=work.parents[1]
methods='''+repr(METHODS)+'''
rows=[]
for index,case in enumerate(json.loads((work/'manifest.json').read_text())):
    for method in methods:
        env=dict(os.environ,XCR_DM_TWO_STAGE='1')
        env.pop('XCR_DM_TRACE_DIR',None)
        env.pop('XCR_BIN_MASK_DIR',None)
        if method!='current': env['XCR_BIN_MASK_DIR']=case['directory']+'/'+method
        run=subprocess.run([str(root/'build/binarization_comparison_20261118/probe'),case['directory']+'/source.gray8'],capture_output=True,text=True,env=env,timeout=15)
        if run.returncode not in (0,1): raise RuntimeError((case,method,run.stderr))
        rows.append(dict(case,method=method,result=json.loads(run.stdout)))
    if (index+1)%20==0: print('Decoded',index+1,flush=True)
(work/'results.json').write_text(json.dumps(rows,indent=2))
'''
    (WORK/'run.py').write_text(runner,encoding='utf-8')
    subprocess.run(['wsl','-e','python3',linux(WORK/'run.py')],check=True)
    rows=json.loads((WORK/'results.json').read_text())
    baseline={r['name']:r['result'] for r in rows if r['method']=='current'}
    good=lambda r:r['decoded']>0 and r['geometry_quality']==2
    summary=[]
    for method in METHODS:
        subset=[r for r in rows if r['method']==method]
        captures=[r for r in subset if r['group']=='capture']
        originals=[r for r in captures if r['dx']==r['dy']==0]
        compare=[r for r in subset if r['group'] in ('historical','extended')]
        summary.append(dict(method=method,
            originals_ok=sum(good(r['result']) and r['result']['payload']=='012348' for r in originals),
            phases_ok=sum(good(r['result']) and r['result']['payload']=='012348' for r in captures),
            lost=[r['name'] for r in compare if good(baseline[r['name']]) and (not good(r['result']) or r['result']['payload']!=baseline[r['name']]['payload'])],
            recovered=[r['name'] for r in compare if not good(baseline[r['name']]) and good(r['result'])],
            negative_false_positive=[r['name'] for r in subset if r['group']=='negative' and r['result']['decoded']>0]))
    report=dict(source_sha256=source_hash,methods=summary,cases=len(cases),replays=len(rows),
                note='Only temporary marker mask replacement; downstream C candidate logic unchanged. Masks precomputed. No P4 performance claim.',rows=rows)
    (OUTPUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    main()
