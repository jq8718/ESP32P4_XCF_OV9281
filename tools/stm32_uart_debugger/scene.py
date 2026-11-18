"""Perspective mesh scene for the independently built UART AGV simulator."""
import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


def add(a, b):
    return (a[0]+b[0], a[1]+b[1], a[2]+b[2])


def mul(a, scale):
    return (a[0]*scale, a[1]*scale, a[2]*scale)


def dot(a, b):
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    return mul(a, 1/max(1e-12, math.sqrt(dot(a, a))))


def body_rotation(yaw, pitch, roll):
    # Side camera: image right=-vehicle X, image down=-Z, optical axis=-Y.
    # R_CQ = Ry(yaw) Rx(pitch) Rz(roll) diag(1,-1,-1).
    # Fixed code + rigid side mount gives body R = Ry(roll) Rx(pitch) Rz(yaw).
    y, p, r = map(math.radians, (yaw, pitch, roll))
    cy,sy,cp,sp,cr,sr = math.cos(y),math.sin(y),math.cos(p),math.sin(p),math.cos(r),math.sin(r)

    def rotate(point):
        x, zside, z = point
        x, zside = x*cy-zside*sy, x*sy+zside*cy
        zside, z = zside*cp-z*sp, zside*sp+z*cp
        x, z = x*cr+z*sr, -x*sr+z*cr
        return (x, zside, z)
    return rotate


def scene_geometry(pose):
    values = pose or {}
    yaw, pitch, roll = (values.get(key) or 0.0 for key in ('yaw_y_deg', 'pitch_x_deg', 'roll_z_deg'))
    rotate = body_rotation(yaw, pitch, roll)
    gap = values.get('distance_to_plane_m')
    if gap is None:
        gap = .55  # Installation illustration only, before valid telemetry.
    along_track = values.get('along_track_m')
    # Measured aisle-forward is -scene X.
    # Rotate around the fixed-height optical centre, so label and lens stay
    # at 25 cm even when the illustrative body tilts.
    anchor = (-(along_track or 0.0)*2.4, gap, .25)
    camera_local = (0.0, -.315, .25)
    world_points = {}

    def world(point):
        if point not in world_points:
            world_points[point] = add(anchor, rotate((point[0]-camera_local[0],point[1]-camera_local[1],point[2]-camera_local[2])))
        return world_points[point]

    return world, anchor, rotate, (yaw, pitch, roll)


class MeshRenderer:
    def __init__(self, painter, width, height, eye, target):
        self.painter = painter
        self.eye = eye
        self.forward = unit(tuple(target[i]-eye[i] for i in range(3)))
        self.right = unit(cross(self.forward, (0., 0., 1.)))
        self.up = cross(self.right, self.forward)
        # Actual vertical FOV; horizontal FOV follows the viewport aspect ratio.
        self.focal = height/(2*math.tan(math.radians(52)/2))
        self.center = (width*.48, height*.51)
        self.faces = []
        self.view_points = {}

    def view(self, point):
        if point not in self.view_points:
            v = (point[0]-self.eye[0],point[1]-self.eye[1],point[2]-self.eye[2])
            self.view_points[point] = (dot(v, self.right), dot(v, self.up), dot(v, self.forward))
        return self.view_points[point]

    def project(self, point):
        x, y, z = self.view(point)
        if z <= .06:
            return None
        return QPointF(self.center[0]+self.focal*x/z, self.center[1]-self.focal*y/z)

    def polygon(self, points, color, lit=True, edge=None):
        points = list(points)
        normal = unit(cross(tuple(points[1][i]-points[0][i] for i in range(3)),
                            tuple(points[2][i]-points[0][i] for i in range(3))))
        fill = QColor(color)
        if lit:
            illumination = .64+.36*abs(dot(normal, unit((-.45, -.6, 1.0))))
            fill = QColor.fromRgbF(fill.redF()*illumination, fill.greenF()*illumination,
                                  fill.blueF()*illumination, fill.alphaF())
        vertices = [self.view(p) for p in points]
        # Clip geometry crossing the eye plane; never clamp negative depth.
        clipped = []
        for previous, current in zip(vertices[-1:]+vertices[:-1], vertices):
            pin, cin = previous[2] > .06, current[2] > .06
            if pin != cin:
                t = (.06-previous[2])/(current[2]-previous[2])
                clipped.append(tuple(previous[i]+t*(current[i]-previous[i]) for i in range(3)))
            if cin:
                clipped.append(current)
        if len(clipped) < 3:
            return
        polygon = QPolygonF([QPointF(self.center[0]+self.focal*x/z,
                                    self.center[1]-self.focal*y/z) for x, y, z in clipped])
        self.faces.append((sum(p[2] for p in clipped)/len(clipped), polygon, fill, edge))

    def flush(self):
        for _, polygon, color, edge in sorted(self.faces, key=lambda item: item[0], reverse=True):
            self.painter.setPen(QPen(QColor(edge), .7) if edge else Qt.PenStyle.NoPen)
            self.painter.setBrush(color)
            self.painter.drawPolygon(polygon)
        self.faces.clear()

    def line(self, a, b, color, width=1.5, dashed=False):
        a, b = self.project(a), self.project(b)
        if a is None or b is None:
            return
        self.painter.setPen(QPen(QColor(color), width, Qt.PenStyle.DashLine if dashed else Qt.PenStyle.SolidLine))
        self.painter.drawLine(a, b)


class AgvView(QWidget):
    def __init__(self):
        super().__init__()
        self.pose = None
        self.setMinimumSize(520, 350)

    def set_pose(self, pose):
        self.pose = dict(pose) if pose else None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        background = QLinearGradient(0, 0, 0, self.height())
        background.setColorAt(0, QColor('#f4f7fa'))
        background.setColorAt(1, QColor('#c9d5df'))
        painter.fillRect(self.rect(), background)
        world, camera, rotate, angles = scene_geometry(self.pose)
        # Rear-right chase view shows both the rear lamps and the right camera.
        # Follow yaw only, keeping the horizon level when the vehicle tilts.
        yaw = math.radians(angles[0])
        ahead = (math.cos(yaw), math.sin(yaw), 0.)
        right = (math.sin(yaw), -math.cos(yaw), 0.)
        center = world((0., 0., .16))
        eye = add(add(center, mul(ahead, -1.48)), add(mul(right, .45), (0., 0., .94)))
        eye = (eye[0], max(.07, min(1.43, eye[1])), eye[2])
        target = add(center, add(mul(ahead, .18), (0., 0., .04)))
        renderer = MeshRenderer(painter, self.width(), self.height(), eye, target)
        face = renderer.polygon

        def mesh(points, color, edge=None):
            face([world(p) for p in points], color, edge=edge)

        def box(origin, size, color, bevel=0.):
            cx, cy, z = origin
            length, width, height = size
            hx, hy = length/2, width/2
            if bevel:
                b = min(bevel, hx*.8, hy*.8)
                outline = [(-hx+b,-hy), (hx-b,-hy), (hx,-hy+b), (hx,hy-b),
                           (hx-b,hy), (-hx+b,hy), (-hx,hy-b), (-hx,-hy+b)]
            else:
                outline = [(-hx,-hy), (hx,-hy), (hx,hy), (-hx,hy)]
            bottom = [(cx+x,cy+y,z) for x,y in outline]
            top = [(cx+x,cy+y,z+height) for x,y in outline]
            for i in range(len(top)):
                j = (i+1) % len(top)
                mesh([bottom[i],bottom[j],top[j],top[i]], color)
            mesh(top, QColor(color).lighter(118))

        def cylinder(origin, radius, length, axis, color, segments=24):
            if axis == 'y':
                circle = lambda a, offset: (origin[0]+radius*math.cos(a), origin[1]+offset,
                                           origin[2]+radius*math.sin(a))
            else:
                circle = lambda a, offset: (origin[0]+radius*math.cos(a),
                                           origin[1]+radius*math.sin(a), origin[2]+offset)
            rings = [[circle(2*math.pi*i/segments, offset) for i in range(segments)]
                     for offset in (-length/2, length/2)]
            mesh(rings[0][::-1], color)
            mesh(rings[1], color)
            for i in range(segments):
                j = (i+1) % segments
                mesh([rings[0][i],rings[0][j],rings[1][j],rings[1][i]], color)

        # Finite wall tiles and floor panels are sorted together with the car.
        for x in range(-4, 6):
            face([(x,0,-.006),(x+1,0,-.006),(x+1,1.5,-.006),(x,1.5,-.006)],
                 '#cbd4dc' if x % 2 else '#d4dde4', lit=False, edge='#bcc8d2')
            for y in (0., 1.5):
                face([(x,y,0),(x+1,y,0),(x+1,y,1.0),(x,y,1.0)],
                     '#e5eaf0' if y == 0 else '#dbe3eb', edge='#bac8d4')
                inward = .002 if y == 0 else -.002
                face([(x,y+inward,.035),(x+1,y+inward,.035),
                      (x+1,y+inward,.075),(x,y+inward,.075)], '#708494', lit=False)
            for y in (.07, 1.41):
                face([(x,y,.001),(x+.75,y,.001),(x+.75,y+.025,.001),(x,y+.025,.001)],
                     '#ecbd4b', lit=False)
        # The opaque aisle is the environment pass; floor polygons cannot
        # incorrectly paint over wheels/body when their mean depths overlap.
        renderer.flush()
        # Code board lies in the wall plane, not in the viewing-camera plane.
        face([(-.11,.005,.14),(.11,.005,.14),(.11,.005,.36),(-.11,.005,.36)],
             '#ffffff', lit=False, edge='#536574')
        rows = ('111111111111', '100101001101', '110011010101', '101101001011',
                '100110111001', '111001010101', '101011001111', '110100101001',
                '100111010101', '111010110011', '101101001101', '111111111111')
        module = .176/12
        for row, pattern in enumerate(rows):
            for col, bit in enumerate(pattern):
                if bit == '1':
                    x, z = -.088+col*module, .338-row*module
                    face([(x,.006,z),(x+module,.006,z),(x+module,.006,z-module),(x,.006,z-module)],
                         '#132838', lit=False)
        renderer.flush()

        # Soft footprint stays on the floor, rather than floating with the body.
        shadow = [world((x,y,0)) for x,y in ((-.48,-.31),(.48,-.31),(.48,.31),(-.48,.31))]
        face([(x,y,.002) for x,y,z in shadow], '#9baebc', lit=False)
        renderer.flush()
        # Four actual 3D tyres; axle direction is transverse to the body.
        for wx in (-.285, .285):
            for wy in (-.218, .218):
                cylinder((wx,wy,.09), .09, .063, 'y', '#202b33', 20)
                outside = wy+math.copysign(.033, wy)
                cylinder((wx,outside,.09), .052, .008, 'y', '#a2b2be', 16)
                cylinder((wx,outside+math.copysign(.005,wy),.09), .022, .009, 'y', '#334c5d', 12)
                for a in range(0, 360, 60):
                    ar = math.radians(a)
                    bx, bz = wx+.035*math.cos(ar), .09+.035*math.sin(ar)
                    mesh([(bx+.005*math.cos(i*math.pi/3), outside+math.copysign(.005,wy),
                           bz+.005*math.sin(i*math.pi/3)) for i in range(6)], '#dde8ef')
        # Low industrial chassis, rounded corner shell and differentiated ends.
        box((0,0,.095), (.91,.50,.075), '#273947', .055)
        box((-.025,0,.162), (.82,.475,.145), '#237aa4', .075)
        box((-.035,0,.302), (.70,.43,.028), '#e5edf2', .075)
        box((.36,0,.165), (.105,.43,.095), '#e8eff3', .035)
        box((-.44,0,.125), (.055,.46,.09), '#1e2f3d', .02)
        box((.44,0,.115), (.048,.45,.06), '#263c4c', .015)
        box((-.11,0,.332), (.32,.29,.01), '#bbcbd6', .015)
        # Rear ventilation slats and two red tail lamps; white headlights up front.
        for y in (-.065,-.039,-.013,.013,.039,.065):
            box((-.471,y,.169), (.006,.012,.03), '#0c1d2b')
        for side in (-1,1):
            box((-.473,.167*side,.167), (.009,.065,.025), '#f05454', .003)
            box((.466,.155*side,.174), (.009,.08,.024), '#f8ffff', .003)
            box((.465,.203*side,.147), (.01,.025,.012), '#ffbd4b')
        # Top direction arrow and white FRONT text distinguish front from rear.
        mesh([(.12,-.065,.334),(.22,-.065,.334),(.22,-.11,.334),
              (.34,0,.334),(.22,.11,.334),(.22,.065,.334),(.12,.065,.334)], '#167a99')
        cylinder((-.13,0,.36), .073, .055, 'z', '#172f40')
        cylinder((-.13,0,.368), .075, .009, 'z', '#31b4cc')
        cylinder((-.29,.105,.347), .029, .014, 'z', '#f2c643')
        cylinder((-.29,.105,.36), .021, .02, 'z', '#e24842')
        cylinder((-.30,-.09,.39), .012, .10, 'z', '#5a7180')
        cylinder((-.30,-.09,.45), .029, .035, 'z', '#ffbd45')
        # Midpoint of the right side, lens centre 25 cm high.
        box((0,-.248,.214), (.15,.035,.072), '#97acb9', .008)
        box((0,-.275,.215), (.12,.045,.07), '#185775', .008)
        cylinder((0,-.302,.25), .027, .012, 'y', '#182a38')
        cylinder((0,-.310,.25), .020, .006, 'y', '#49c5de')
        cylinder((-.006,-.314,.256), .007, .002, 'y', '#d2f7ff', 12)

        # Blue rectangle is the scan field on the wall. All four points share
        # the wall's Y coordinate; the scan-head face is also a real 3D plane.
        wall_field = []
        lens = world((0,-.315,.25))
        for u,v in ((0,0),(1280,0),(1280,720),(0,720)):
            ray = rotate((-(u-589.2736)/733.242515, -1., -(v-359.376026)/732.511533))
            if ray[1] >= -1e-4:
                wall_field = []
                break
            travel = (.009-lens[1])/ray[1]
            if not 0 < travel < 4:
                wall_field = []
                break
            wall_field.append(add(lens, mul(ray, travel)))
        if wall_field:
            # Transparent wall-plane tint participates in geometry depth sorting.
            face(wall_field, '#153a98cb', lit=False)
        renderer.flush()
        if wall_field:
            for a,b in zip(wall_field, wall_field[1:]+wall_field[:1]):
                renderer.line(a,b,'#287cbe',1.3)
        renderer.line(lens,(0,.009,.25),'#ed9c27',1.6,True)
        front = world((.48,0,.24))

        def tag(point, text, color, offset):
            projected = renderer.project(point)
            if projected is None:
                return
            x, y = projected.x()+offset[0], projected.y()+offset[1]
            font = QFont('Microsoft YaHei UI', 9, QFont.Weight.DemiBold)
            painter.setFont(font)
            w = painter.fontMetrics().horizontalAdvance(text)+16
            x = max(8,min(self.width()-w-8,x))
            y = max(66,min(self.height()-94,y))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor('#eef7fbff'))
            painter.drawRoundedRect(QRectF(x,y,w,24),6,6)
            painter.setPen(QColor(color))
            painter.drawText(QPointF(x+8,y+17),text)
        tag(front,'车头 ↑','#126d86',(8,-22))
        wall_gap = self.pose.get('distance_to_plane_m') if self.pose else None
        if wall_gap is not None:
            tag((0,.01,.39),f'{wall_gap * 100:.1f} cm','#425f75',(8,-20))

        # Screen-space labels are painted last, so walls cannot cover the HUD.
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor('#eaf8fbfd'))
        painter.drawRoundedRect(QRectF(10,10,self.width()-20,48),8,8)
        painter.setFont(QFont('Microsoft YaHei UI',10,QFont.Weight.DemiBold))
        painter.setPen(QColor('#24435b'))
        painter.drawText(22,30,'后右方追车视角  ·  过道 1.50 m  ·  AGV 宽 0.50 m')
        painter.setFont(QFont('Microsoft YaHei UI',8))
        painter.setPen(QColor('#637d91'))
        painter.drawText(22,47,'蓝框：与墙面平行的相机视场投影  ·  扫码视场约 82° × 52°')
        painter.setBrush(QColor('#ecf5f9fc'))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(QRectF(10,self.height()-79,self.width()-20,69),8,8)
        painter.setFont(QFont('Microsoft YaHei UI',9))
        painter.setPen(QColor('#28485f'))
        if self.pose:
            p = self.pose
            track = p.get('along_track_m')
            track_text = f'{track:+.3f} m' if track is not None else '—'
            def shown(key, digits=2):
                return '—' if p.get(key) is None else f'{p[key]:+.{digits}f}'
            painter.drawText(22,self.height()-58,
                             f"平滑姿态：yaw {shown('yaw_y_deg')}°  ·  pitch {shown('pitch_x_deg')}°  ·  roll {shown('roll_z_deg')}°")
            painter.drawText(22,self.height()-39,
                             f"平滑距离：离墙 {shown('distance_to_plane_m',3)} m  ·  沿过道 {track_text}  ·  tx {shown('tx_m',3)} m")
            painter.setPen(QColor('#688094'))
            painter.drawText(22,self.height()-21,
                '精简模式：离墙使用平滑后的码平面垂直距离；俯仰/横滚未上传。' if p.get('compact') else
                '姿态和距离均平滑后显示；沿过道位移在场景中放大 2.4 倍。')
        else:
            painter.drawText(22,self.height()-48,'等待串口姿态数据 · 当前为安装位置示意')
        painter.end()
