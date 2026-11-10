import json
import math
from pathlib import Path
import cv2
import numpy as np


def image_rotation(corners):
    horizontal = corners[1][0] - corners[0][0]
    vertical = corners[1][1] - corners[0][1]
    if horizontal == vertical == 0:
        return None
    return math.degrees(math.atan2(vertical, horizontal))


def assumed_calibration(width, height, horizontal_fov=23):
    if width <= 0 or height <= 0 or not 10 <= horizontal_fov <= 150:
        raise ValueError('无效图像尺寸或假定视场角')
    focal = width / (2 * math.tan(math.radians(horizontal_fov) / 2))
    return {'camera_matrix': np.array([[focal, 0, (width - 1) / 2],
                                      [0, focal, (height - 1) / 2], [0, 0, 1]], dtype=np.float64),
            'dist_coeffs': np.zeros(5), 'image_size': [width, height], 'qr_size_mm': None}


def opposite_edge_lengths(corners):
    left = math.dist(corners[0], corners[3])
    right = math.dist(corners[1], corners[2])
    return {'p0_p3_px': left, 'p1_p2_px': right,
            'left_right_ratio': left / right if right > 0 else None}


def load_calibration(path):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    matrix = np.asarray(data['camera_matrix'], dtype=np.float64)
    distortion = np.asarray(data.get('dist_coeffs', [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1)
    size = data['image_size']
    side = data.get('qr_size_mm')
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError('camera_matrix 必须为有效的3×3内参矩阵')
    if not np.allclose(matrix[2], [0, 0, 1]) or abs(matrix[0, 1]) > 1e-9:
        raise ValueError('仅支持标准 OpenCV 相机内参')
    if len(distortion) not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError('dist_coeffs 格式错误')
    if len(size) != 2 or any(int(value) != value or value <= 0 for value in size):
        raise ValueError('image_size 必须为标定图像的宽、高')
    if side is not None and (not isinstance(side, (int, float)) or not math.isfinite(side) or side <= 0):
        raise ValueError('qr_size_mm 必须为二维码有效方形边长，单位毫米')
    return {'camera_matrix': matrix, 'dist_coeffs': distortion, 'image_size': list(size), 'qr_size_mm': side}


def estimate_pose(corners, width, height, calibration, max_reprojection_px=5):
    if calibration is None:
        return {'available': False, 'reason': '未标定：只能显示画面内旋转角'}
    if calibration['image_size'] != [width, height]:
        return {'available': False, 'reason': '标定分辨率与当前图像不一致'}
    image_points = np.asarray(corners, dtype=np.float64)
    if not cv2.isContourConvex(image_points.astype(np.float32)) or abs(cv2.contourArea(image_points.astype(np.float32))) < 16:
        return {'available': False, 'reason': '角点退化，无法估计姿态'}
    image_points = np.roll(image_points, -int(np.argmin(image_points.sum(axis=1))), axis=0)
    side = calibration['qr_size_mm'] or 1.0
    half = side / 2
    object_points = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], dtype=np.float64)
    try:
        solutions = cv2.solvePnPGeneric(object_points, image_points, calibration['camera_matrix'],
                                       calibration['dist_coeffs'], flags=cv2.SOLVEPNP_IPPE)
        candidates = []
        for rotation, translation in zip(solutions[1], solutions[2]):
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                continue
            rotation, translation = cv2.solvePnPRefineLM(object_points, image_points, calibration['camera_matrix'],
                                                        calibration['dist_coeffs'], rotation, translation)
            matrix, _ = cv2.Rodrigues(rotation)
            depths = (matrix @ object_points.T + translation.reshape(3, 1))[2]
            if np.any(depths <= 0):
                continue
            projected, _ = cv2.projectPoints(object_points, rotation, translation, calibration['camera_matrix'], calibration['dist_coeffs'])
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
            if not math.isfinite(error) or not np.isfinite(matrix).all() or not np.isfinite(translation).all():
                continue
            candidates.append((error, matrix, translation.reshape(3)))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > max_reprojection_px:
            return {'available': False, 'reason': '姿态拟合误差过大，请核对标定与角点'}
        error, matrix, translation = candidates[0]
        normal = matrix[:, 2]
        if normal[2] < 0:
            normal = -normal
        angle = math.degrees(math.acos(float(np.clip(normal[2], -1, 1))))
        distinct = [candidate for candidate in candidates[1:] if
                    np.linalg.norm(candidate[1][:, 2] - matrix[:, 2]) > 1e-3]
        return {'available': True, 'front_angle_deg': angle,
                'horizontal_tilt_deg': math.degrees(math.atan2(float(normal[0]), float(normal[2]))),
                'vertical_tilt_deg': math.degrees(math.atan2(float(normal[1]), float(normal[2]))),
                'reprojection_px': error, 'ambiguous': bool(distinct) and distinct[0][0] - error < 0.2,
                'optical_distance_mm': float(translation[2]) if calibration['qr_size_mm'] else None}
    except cv2.error:
        return {'available': False, 'reason': '姿态计算失败，请检查标定数据'}
