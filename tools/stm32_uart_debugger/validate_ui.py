"""Deterministic smoothing/mount checks and UI snapshots; never opens a port."""
import json
import math
from argparse import Namespace
from pathlib import Path
import time

from PySide6.QtWidgets import QApplication
from app import MainWindow, configure_application, parse_pose_line
from pose_filter import POSE_FIELDS, PoseSmoother
from scene import AgvView, scene_geometry, dot


def main():
    pose = parse_pose_line(b'012345,5213,2873,5749,2943,5695,3469,5152,3407,234,-7.69,-7.21,6.65,0.407,0.406,0.400,-0.024,-0.030')
    legacy = parse_pose_line(b'012345,5213,2873,5749,2943,5695,3469,5152,3407,234,-7.69,-7.21,6.65,0.407,0.406,0.400,-0.024')
    assert legacy['along_track_m'] is None and legacy['pose_valid']
    original = dict(pose)
    target = dict(pose)
    for field in POSE_FIELDS:
        target[field] += 1.0 if field.endswith('_deg') else .1
    target['along_track_m'] += .1
    smoother = PoseSmoother()
    smoother.set_target(pose, 0.)
    smoother.set_target(target, 1.)
    filtered = smoother.advance(1.45)
    for key in POSE_FIELDS:
        expected = target[key]+(pose[key]-target[key])*math.exp(-1)
        assert math.isclose(filtered[key], expected, abs_tol=1e-10), key
    expected_track = target['along_track_m']+(pose['along_track_m']-target['along_track_m'])*math.exp(-1)
    assert math.isclose(filtered['along_track_m'], expected_track, abs_tol=1e-10)
    assert pose == original, 'raw UART values were mutated'
    # A fixed time constant must give the same result at different UI frame rates.
    stepped = PoseSmoother()
    stepped.set_target(pose, 0.)
    stepped.set_target(target, 1.)
    for tick in range(1, 61):
        current = stepped.advance(1+tick*.45/60)
    assert all(abs(current[key]-filtered[key]) < 1e-10 for key in POSE_FIELDS)
    smoother.reset()
    smoother.set_target(dict(pose, **{key:179. for key in POSE_FIELDS[:3]}), 0.)
    smoother.set_target(dict(pose, **{key:-179. for key in POSE_FIELDS[:3]}), 1.)
    wrapped = smoother.advance(1.2)
    assert all(abs(wrapped[key]) > 179 for key in POSE_FIELDS[:3])
    smoother.set_target(dict(pose, pose_valid=False), 2.)
    assert smoother.advance(3.) is None

    for angles in ((0,0,0),(-12,8,5),(40,-10,20)):
        sample = dict(pose, **dict(zip(POSE_FIELDS[:3], angles)))
        world, anchor, rotate, _ = scene_geometry(sample)
        lens = world((0.,-.315,.25))
        assert abs(lens[2]-.25) < 1e-12
        assert abs(lens[1]-sample['distance_to_plane_m']) < 1e-12
        assert abs(lens[0]+sample['along_track_m']*2.4) < 1e-12
        axes = [rotate(axis) for axis in ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))]
        for i in range(3):
            for j in range(3):
                assert abs(dot(axes[i],axes[j])-(1. if i == j else 0.)) < 1e-12
        if angles == (0,0,0):
            # Scan-head front is a wall-parallel X/Z plane in neutral pose.
            assert abs(world((.06,-.315,.25))[1]-lens[1]) < 1e-12
            assert abs(world((0,-.315,.285))[1]-lens[1]) < 1e-12
            assert abs(world((0,-.218,0))[2]) < 1e-12, 'neutral wheel must touch ground'
    # Same camera centre, different yaw and raw tx: fixed aisle position stays put.
    spin = dict(pose, yaw_y_deg=10., tx_m=.0868, optical_depth_m=.4924, along_track_m=0.)
    _, anchor, _, _ = scene_geometry(spin)
    assert abs(anchor[0]) < 1e-12

    out = Path(__file__).parent/'validation/scene_revision'
    out.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    configure_application(app)
    view = AgvView()
    view.resize(850,540)
    view.show()
    for name, sample in (('neutral',dict(pose, yaw_y_deg=0.,pitch_x_deg=0.,roll_z_deg=0.)),
                         ('tilted',pose),('waiting',None)):
        view.set_pose(sample)
        app.processEvents()
        assert view.grab().save(str(out/f'{name}.png'))
    timings = []
    for _ in range(12):
        start = time.perf_counter()
        view.set_pose(pose)
        view.grab()
        timings.append((time.perf_counter()-start)*1000)
    view.close()
    window = MainWindow(Namespace(port=None,auto_connect=False,smoke_seconds=0,smoke_output=''))
    window.show()
    app.processEvents()
    window.receive_result(pose)
    window.receive_result(target)
    window.pose_smoother.advance(time.monotonic()+.45)
    window.update_scene()
    for key in POSE_FIELDS:
        assert abs(window.simulation.pose[key]-window.pose_smoother.current[key]) < 1e-10
    assert window.values['tx_m'][0].text() == f"{target['tx_m']:+.3f} m"
    assert window.values['along_track_m'][0].text() == f"{target['along_track_m']:+.3f} m"
    assert window.grab().save(str(out/'window.png'))
    window.close()
    report = {'status':'passed','checks':['eight-field EMA','legacy UART compatibility','frame-rate independence','angle wrap',
              'raw data unchanged','invalid pose reset','rigid side camera mount','25 cm optical height',
              'neutral scan-head wall alignment','wheel ground contact','in-place yaw position invariant','UI filtered pose wiring'],
              'mean_scene_paint_ms':sum(timings)/len(timings)}
    (out/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
