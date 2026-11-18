"""Deterministic smoothing/mount checks and UI snapshots; never opens a port."""
import json
import math
from argparse import Namespace
from pathlib import Path
import time

from PySide6.QtWidgets import QApplication
from app import MainWindow, CalibrationDialog, configure_application, parse_pose_line
from uart_protocol import frame, FACTORY_CAL, parse_reply, CAL_NAMES
from pose_filter import POSE_FIELDS, PoseSmoother
from scene import AgvView, scene_geometry, dot


def main():
    pose = parse_pose_line(b'@SCAN,6,F,999,1,012345,5213,2873,5749,2943,5695,3469,5152,3407,-7690,-7210,6650,407,406,400,-24,-30*E379\r\n')
    assert pose['payload'] == '012345'
    for old in (b'012345,5213,2873,5749,2943,5695,3469,5152,3407,234,-7.69,-7.21,6.65,0.407,0.406,0.400,-0.024',
                frame('@SCAN,1,C,234,1,30,-7690,-30'),
                frame('@SCAN,2,C,234,1,30,-7690,406,-30'),
                frame('@SCAN,3,C,234,1,30,-7690,406,400,-30'),
                frame('@SCAN,4,C,235,1,303132333435,-7690,375,-30')):
        try:
            parse_pose_line(old)
        except ValueError:
            pass
        else:
            raise AssertionError('obsolete protocol accepted')
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
    compact = parse_pose_line(b'@SCAN,6,C,0,1,012345,-7690,375,-30*E08E\r\n')
    assert compact['payload'] == '012345'
    partial = PoseSmoother()
    partial.set_target(pose, 0.)
    partial.set_target(compact, 1.)
    reduced = partial.advance(1.5)
    assert reduced['pitch_x_deg'] is None and reduced['range_m'] is None
    assert reduced['optical_depth_m'] is None
    assert math.isclose(reduced['distance_to_plane_m'], .375+.025*math.exp(-.5/.45))
    _, compact_anchor, _, _ = scene_geometry(compact)
    assert compact_anchor[1] == .375
    # Optical depth changes under in-place yaw; measured wall clearance stays fixed.
    for yaw in (0., 30., -30.):
        turn = dict(compact, yaw_y_deg=yaw, optical_depth_m=.5*math.cos(math.radians(yaw)),
                    distance_to_plane_m=.5, along_track_m=0.)
        _, anchor, _, _ = scene_geometry(turn)
        assert anchor == (0., .5, .25)
    _, closer, _, _ = scene_geometry(dict(compact, distance_to_plane_m=.25))
    assert closer[1] == .25
    assert reduced['yaw_y_deg'] is not None and reduced['along_track_m'] is not None
    partial.set_target(pose, 2.)
    assert partial.advance(2.1)['range_m'] == pose['range_m']
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
    assert not window.full_upload.isChecked()
    assert window.history.columnWidth(1) < 90
    assert window.history.columnWidth(1) >= window.history.fontMetrics().horizontalAdvance('888888') + 10
    commands = []
    window.send_command = lambda text: commands.append(text) or True
    window.full_upload.setChecked(True)
    assert commands[-1] == '@OUT,FULL'
    window.receive_reply(dict(kind='output', mode='FULL'))
    assert '已确认' in window.mode_status.text()
    window.full_upload.setChecked(False)
    assert commands[-1] == '@OUT,COMPACT'
    window.receive_reply(dict(kind='output', mode='COMPACT'))
    window.receive_result(compact)
    assert [window.history.item(i, 0).text() for i in (1, 2)] == ['999', '0']
    assert window.values['range_m'][0].text() == '—'
    assert window.values['optical_depth_m'][0].text() == '—'
    assert window.values['distance_to_plane_m'][0].text() == '0.375 m'
    assert window.simulation.pose['distance_to_plane_m'] is not None
    assert scene_geometry(window.simulation.pose)[1][1] == window.pose_smoother.current['distance_to_plane_m']
    assert '#086cac' in window.content.styleSheet()
    assert window.values['along_track_m'][0].text() == '-0.030 m'
    assert '未上传' in window.frame_info.text()
    app.processEvents()
    assert window.grab().save(str(out/'compact.png'))
    dialog = CalibrationDialog(window)
    dialog.show()
    dialog.read_device()
    assert commands[-1] == '@CAL,GET'
    reply = parse_reply(('@CAL,DATA,'+','.join(str(FACTORY_CAL[k]) for k in CAL_NAMES)).encode())
    dialog.response(reply)
    assert '已读取' in dialog.message.text()
    dialog.inputs['fx'].setValue(734.25)
    dialog.download()
    assert commands[-1].startswith('@CAL,SET,1280,720,734.25,')
    changed = dict(FACTORY_CAL, fx=734.25)
    dialog.response(dict(kind='calibration', values=changed))
    assert '下载成功' in dialog.message.text()
    dialog.download()
    dialog.response(dict(kind='error', command='@CAL', reason='flash'))
    assert 'flash' in dialog.message.text()
    dialog.read_device()
    dialog.command_timeout()
    assert '无法判断' in dialog.message.text()
    app.processEvents()
    assert dialog.grab().save(str(out/'calibration.png'))
    dialog.close()
    window.close()
    report = {'status':'passed','checks':['eight-field EMA','obsolete UART rejected','frame-rate independence','angle wrap',
              'raw data unchanged','invalid pose reset','rigid side camera mount','25 cm optical height',
              'neutral scan-head wall alignment','wheel ground contact','in-place yaw position invariant',
              'UI filtered pose wiring','compact/full switch commands and acknowledgements', 'frame number wraps in history',
              'compact wall position uses smoothed plane distance', 'yaw invariant wall clearance',
              'missing compact fields cleared','calibration GET/SET/confirmation/error/timeout'],
              'mean_scene_paint_ms':sum(timings)/len(timings)}
    (out/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
