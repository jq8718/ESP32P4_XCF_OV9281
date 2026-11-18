"""WSL experiment: rethreshold only rejected elongated components.

Existing mean windows and downstream C code are preserved. Precomputed
masks are injected with the correct radius for each real C search pass.
This tests detection, not the cost of a future on-device implementation.
"""
import subprocess
import cv2
import numpy as np
import check_morphology_phases as runner


def split_components(gray,radius,details=None):
    n=radius*2+1
    sums=cv2.boxFilter(gray.astype(np.float64),-1,(n,n),normalize=False,borderType=cv2.BORDER_CONSTANT)
    areas=cv2.boxFilter(np.ones(gray.shape,np.float64),-1,(n,n),normalize=False,borderType=cv2.BORDER_CONSTANT)
    mask=(gray.astype(np.float64)*areas*100<sums*75).astype(np.uint8)
    total,labels,stats,_=cv2.connectedComponentsWithStatsWithAlgorithm(mask,4,cv2.CV_32S,cv2.CCL_WU)
    attempts=0;work=0
    for label in range(1,total):
        x,y,w,h,area=map(int,stats[label])
        if min(w,h)<2 or max(w,h)>64 or max(w,h)*6<=min(w,h)*10 or area<8: continue
        if work+area>4096 or attempts>=8: continue
        attempts+=1;work+=area
        local_labels=labels[y:y+h,x:x+w]
        region=local_labels==label
        local_gray=gray[y:y+h,x:x+w]
        values=local_gray[region]
        cutoff,_=cv2.threshold(values.reshape(-1,1),0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        dark=values[values<=cutoff];light=values[values>cutoff]
        if len(dark)<4 or len(light)<4 or float(light.mean())-float(dark.mean())<12: continue
        local_mask=mask[y:y+h,x:x+w]
        local_mask[region]=(values<=cutoff).astype(np.uint8)
        if details is not None:
            details.append(dict(box=[x,y,w,h],original_points=area,cutoff=float(cutoff),
                dark_points=len(dark),paper_points=len(light),dark_mean=float(dark.mean()),
                paper_mean=float(light.mean()),window_radius=radius))
    return mask


def prepare(image):
    for step in (6,4):
        gray=image[::step,::step]
        for radius in (6,2):
            folder=runner.WORK/'component_otsu'
            folder.mkdir(exist_ok=True)
            split_components(gray,radius).tofile(folder/f'mask_{step}_r{radius}.gray8')


def main():
    runner.WORK=runner.ROOT/'build/component_rethreshold_20261118'
    runner.WORK.mkdir(parents=True,exist_ok=True)
    source=(runner.ROOT/'build/binarization_comparison_20261118/probe_marker.c').read_text()
    old='"%s/mask_%d.gray8", mask_directory, step'
    assert source.count(old)==1
    source=source.replace(old,'"%s/mask_%d_r%d.gray8", mask_directory, step, radius')
    path=runner.WORK/'marker.c'
    path.write_text(source)
    common=['gcc','-O2','-std=c11','-fno-strict-aliasing','-DCONFIG_XCR_DATAMATRIX_LIBDMTX=0','-DXCR_DM_TRACE_STAGES=1',
        '-Iport/components/xcr_decoder/include','-Iport/components/xcr_decoder/vendor/include','-Iport/components/xcr_decoder/vendor/quirc']
    subprocess.run(common+['-c',str(path),'-o',str(runner.WORK/'marker.o')],cwd=runner.ROOT,check=True)
    objects=[runner.ROOT/'build/binarization_comparison_20261118'/f'{name}.o' for name in ('main','port','core')]
    runner.PROBE=runner.WORK/'probe'
    subprocess.run(['gcc',str(runner.WORK/'marker.o')]+list(map(str,objects))+['-lm','-o',str(runner.PROBE)],check=True)
    runner.METHODS=('current','component_otsu')
    runner.REPORT_FILENAME='component_report.json'
    runner.prepare=prepare
    runner.main()


if __name__=='__main__': main()
