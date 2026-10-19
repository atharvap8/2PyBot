"""Camera geometry only. No camera ownership, ISP writes or robot commands."""
import base64
import collections
import math

import cv2
import numpy as np

cv2.setNumThreads(1)
_REMAP_CACHE = collections.OrderedDict()


def board_settings(values=None):
    values = values or {}
    result = {"kind": str(values.get("kind", "charuco")),
              "columns": int(values.get("columns", 8)), "rows": int(values.get("rows", 6)),
              "square_mm": float(values.get("square_mm", 20)),
              "marker_ratio": float(values.get("marker_ratio", .70))}
    if result["kind"] not in ("charuco", "checkerboard"):
        raise ValueError("Choose ChArUco or checkerboard")
    if not 4 <= result["columns"] <= 14 or not 4 <= result["rows"] <= 12:
        raise ValueError("Board dimensions must be 4–14 columns and 4–12 rows")
    if result["kind"] == "charuco" and (result["columns"]*result["rows"]+1)//2 > 50:
        raise ValueError("This ChArUco dictionary supports up to 50 markers; reduce the board dimensions")
    if not math.isfinite(result["square_mm"]) or not 5 <= result["square_mm"] <= 60:
        raise ValueError("Square size must be 5–60 mm")
    if not math.isfinite(result["marker_ratio"]) or not .4 <= result["marker_ratio"] <= .85:
        raise ValueError("Marker size must be 40–85% of a square")
    return result


def charuco_board(settings):
    if not hasattr(cv2, "aruco"):
        raise ValueError("OpenCV ArUco support is required for ChArUco; choose checkerboard or install opencv-contrib")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    size = settings["square_mm"] / 1000
    if hasattr(aruco, "CharucoBoard_create"):
        board = aruco.CharucoBoard_create(settings["columns"], settings["rows"], size,
                                         size * settings["marker_ratio"], dictionary)
    else:
        board = aruco.CharucoBoard((settings["columns"], settings["rows"]), size,
                                  size * settings["marker_ratio"], dictionary)
        if hasattr(board, "setLegacyPattern"):
            board.setLegacyPattern(True)  # same printed board on OpenCV 4.5 and newer
    return board, dictionary


def board_points(settings):
    if settings["kind"] == "charuco":
        board, _ = charuco_board(settings)
        return np.asarray(board.getChessboardCorners() if hasattr(board, "getChessboardCorners")
                          else board.chessboardCorners, dtype=np.float32)
    points = np.zeros(((settings["columns"]-1)*(settings["rows"]-1), 3), np.float32)
    points[:, :2] = np.mgrid[0:settings["columns"]-1, 0:settings["rows"]-1].T.reshape(-1, 2)
    return points * settings["square_mm"] / 1000


def board_png(settings, pixels=1200):
    height = round(pixels * settings["rows"] / settings["columns"])
    if settings["kind"] == "charuco":
        board, _ = charuco_board(settings)
        image = board.generateImage((pixels, height), marginSize=0) if hasattr(board, "generateImage") else board.draw((pixels, height), marginSize=0)
    else:
        image = np.full((height, pixels), 255, np.uint8)
        for row in range(settings["rows"]):
            for column in range(settings["columns"]):
                if (row+column) % 2 == 0:
                    image[round(row*height/settings["rows"]):round((row+1)*height/settings["rows"]),
                          round(column*pixels/settings["columns"]):round((column+1)*pixels/settings["columns"])] = 0
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("Could not render board")
    return encoded.tobytes()


def board_svg(settings):
    width = settings["columns"] * settings["square_mm"]
    height = settings["rows"] * settings["square_mm"]
    page_width, page_height = max(210, width+30), max(297, height+45)
    image = base64.b64encode(board_png(settings)).decode()
    label = f'{settings["kind"]} · {settings["columns"]}×{settings["rows"]} squares · {settings["square_mm"]:g} mm'
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_width}mm" height="{page_height}mm" '
            f'viewBox="0 0 {page_width} {page_height}"><rect width="100%" height="100%" fill="white"/>'
            f'<image x="{(page_width-width)/2}" y="20" width="{width}" height="{height}" '
            f'href="data:image/png;base64,{image}"/><text x="15" y="{height+32}" font-size="4">{label}</text>'
            f'<text x="15" y="{height+40}" font-size="3.5">Print at 100% / actual size. Measure one square before capturing.</text>'
            f'<path d="M15 {height+52} h50 m-50 -2 v4 m50 -4 v4" stroke="black" fill="none"/>'
            f'<text x="15" y="{height+59}" font-size="3.5">This line must measure 50 mm.</text></svg>').encode()


def detect(image, settings):
    h, w = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    factor = min(1.0, 1100/w)
    small = cv2.resize(gray, None, fx=factor, fy=factor, interpolation=cv2.INTER_AREA) if factor < 1 else gray
    ids = None
    if settings["kind"] == "charuco":
        board, dictionary = charuco_board(settings)
        if hasattr(cv2.aruco, "CharucoDetector"):
            corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(small)
        else:
            markers, marker_ids, _ = cv2.aruco.detectMarkers(small, dictionary)
            corners = None
            if marker_ids is not None:
                _, corners, ids = cv2.aruco.interpolateCornersCharuco(markers, marker_ids, small, board)
        if corners is None or ids is None or len(corners) < 12:
            return None, {"detected": False, "message": "Show the printed ChArUco board; keep its markers visible"}
        corners = np.asarray(corners, np.float32).reshape(-1, 1, 2) / factor
        objects = board_points(settings)[np.asarray(ids).reshape(-1)]
    else:
        pattern = (settings["columns"]-1, settings["rows"]-1)
        found, corners = cv2.findChessboardCorners(small, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return None, {"detected": False, "message": "Show the entire checkerboard, including all four edges"}
        corners = np.asarray(corners, np.float32) / factor
        objects = board_points(settings)
    cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, .03))
    points = corners.reshape(-1, 2)
    hull = cv2.convexHull(points)
    area = float(cv2.contourArea(hull)/(w*h))
    centre = points.mean(axis=0)/[w, h]
    x0, y0 = np.maximum(0, np.floor(points.min(axis=0)-10)).astype(int)
    x1, y1 = np.minimum([w, h], np.ceil(points.max(axis=0)+10)).astype(int)
    patch = gray[y0:y1, x0:x1]
    sharpness = float(cv2.Laplacian(patch, cv2.CV_32F).var()) if patch.size else 0
    brightness = float(patch.mean()) if patch.size else 0
    homography, _ = cv2.findHomography(objects[:, :2], points)
    tilted = False
    rotation = 0.0
    if homography is not None:
        bounds = np.array([[0, 0], [settings["columns"]*settings["square_mm"]/1000, 0],
                           [settings["columns"]*settings["square_mm"]/1000, settings["rows"]*settings["square_mm"]/1000],
                           [0, settings["rows"]*settings["square_mm"]/1000]], np.float32).reshape(-1, 1, 2)
        quad = cv2.perspectiveTransform(bounds, homography).reshape(-1, 2)
        lengths = [np.linalg.norm(quad[(i+1)%4]-quad[i]) for i in range(4)]
        tilted = min(lengths[0], lengths[2])/max(lengths[0], lengths[2], 1e-6) < .82 or min(lengths[1], lengths[3])/max(lengths[1], lengths[3], 1e-6) < .82
        rotation = float(math.degrees(math.atan2(*(quad[1]-quad[0])[::-1])))
    metrics = {"detected": True, "corners": len(points), "centre": centre.tolist(), "area": area,
               "sharpness": round(sharpness, 1), "brightness": round(brightness, 1),
               "tilted": bool(tilted), "rotation_deg": round(rotation, 1),
               "cell": min(2, int(centre[1]*3))*3+min(2, int(centre[0]*3)),
               "scale_bin": 0 if area < .10 else 1 if area < .22 else 2,
               "border_margin": float(min(points[:, 0].min()/w, points[:, 1].min()/h,
                                           1-points[:, 0].max()/w, 1-points[:, 1].max()/h))}
    return {"object_points": objects.tolist(), "image_points": points.tolist(),
            "corner_ids": np.asarray(ids).reshape(-1).tolist() if ids is not None else list(range(len(points)))}, metrics


def reprojection_error(objects, pixels, matrix, distortion, model):
    objects = np.asarray(objects, np.float64).reshape(-1, 1, 3)
    pixels = np.asarray(pixels, np.float64).reshape(-1, 1, 2)
    if model == "fisheye":
        normalized = cv2.fisheye.undistortPoints(pixels, matrix, distortion)
        ok, rotation, translation = cv2.solvePnP(objects, normalized, np.eye(3), None)
    else:
        ok, rotation, translation = cv2.solvePnP(objects, pixels, matrix, distortion)
    if not ok:
        raise ValueError("Target pose could not be estimated")
    projected, _ = (cv2.fisheye.projectPoints(objects, rotation, translation, matrix, distortion)
                    if model == "fisheye" else cv2.projectPoints(objects, rotation, translation, matrix, distortion))
    residual = projected.reshape(-1, 2)-pixels.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))), residual


def fit_model(samples, image_size, model):
    objects = [np.asarray(s["object_points"], np.float32).reshape(-1, 1, 3) for s in samples]
    pixels = [np.asarray(s["image_points"], np.float32).reshape(-1, 1, 2) for s in samples]
    if model == "fisheye":
        matrix = np.array([[image_size[0], 0, image_size[0]/2], [0, image_size[0], image_size[1]/2], [0, 0, 1]], np.float64)
        distortion = np.zeros((4, 1), np.float64)
        rms, matrix, distortion, _, _ = cv2.fisheye.calibrate(
            [p.astype(np.float64) for p in objects], [p.astype(np.float64) for p in pixels], tuple(image_size), matrix, distortion,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7))
    else:
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, pixels, tuple(image_size), None, None)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or not math.isfinite(rms):
        raise ValueError("Calibration produced non-finite values")
    if not .1*image_size[0] <= matrix[0, 0] <= 5*image_size[0] or not .1*image_size[1] <= matrix[1, 1] <= 8*image_size[1]:
        raise ValueError("Estimated focal length is implausible; capture more varied poses")
    if not -.1*image_size[0] <= matrix[0, 2] <= 1.1*image_size[0] or not -.1*image_size[1] <= matrix[1, 2] <= 1.1*image_size[1]:
        raise ValueError("Estimated optical centre is implausible; capture edge and tilted views")
    return matrix, distortion, float(rms)


def solve(samples, image_size):
    if len(samples) < 20:
        raise ValueError("At least 20 diverse accepted views are required")
    heldout_indices = set(range(4, len(samples), 5))
    training = [s for i, s in enumerate(samples) if i not in heldout_indices]
    heldout = [s for i, s in enumerate(samples) if i in heldout_indices]
    candidates = []
    for model in ("pinhole", "fisheye"):
        try:
            matrix, distortion, rms = fit_model(training, image_size, model)
            errors = [reprojection_error(s["object_points"], s["image_points"], matrix, distortion, model)[0] for s in heldout]
            candidates.append({"model": model, "training_rms_px": rms,
                               "heldout_mean_px": float(np.mean(errors)), "heldout_p95_px": float(np.percentile(errors, 95)),
                               "heldout_errors_px": errors, "eligible": True})
        except (cv2.error, ValueError) as exc:
            candidates.append({"model": model, "eligible": False, "error": str(exc)[-500:]})
    usable = [c for c in candidates if c["eligible"]]
    if not usable:
        raise ValueError("Both lens models failed; add sharp tilted views at different positions")
    selected = min(usable, key=lambda c: c["heldout_mean_px"])
    model = selected["model"]
    matrix, distortion, rms = fit_model(samples, image_size, model)
    errors = []
    residuals = []
    for sample in samples:
        error, residual = reprojection_error(sample["object_points"], sample["image_points"], matrix, distortion, model)
        errors.append(error); residuals.extend(residual.tolist())
    passed = selected["heldout_mean_px"] <= 2.5 and selected["heldout_p95_px"] <= 4.0 and rms <= 2.5
    if model == "fisheye":
        rectified = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(matrix, distortion, tuple(image_size), np.eye(3), balance=.25)
        roi = [0, 0, image_size[0], image_size[1]]
    else:
        rectified, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, tuple(image_size), .25, tuple(image_size))
    return {"schema": "2pybot-camera-intrinsics/1", "model": model, "image_size": list(image_size),
            "camera_matrix": matrix.tolist(), "distortion": distortion.reshape(-1).tolist(),
            "rectified_matrix": rectified.tolist(), "valid_roi": list(map(int, roi)),
            "length_unit": "m", "optical_axes": "X right / Y down / Z forward",
            "quality": {"passed": passed, "rms_px": rms, "heldout_mean_px": selected["heldout_mean_px"],
                        "heldout_p95_px": selected["heldout_p95_px"], "per_view_errors_px": errors,
                        "residual_p95_px": float(np.percentile(np.linalg.norm(residuals, axis=1), 95)),
                        "training_views": len(training), "heldout_views": len(heldout), "total_views": len(samples),
                        "thresholds_px": {"mean": 2.5, "p95": 4.0}, "model_comparison": candidates,
                        "note": "Held-out target reprojection, not a physical straight-line/pose validation guarantee"}}


def rectify(image, result, output_width=1100):
    source_size = result["image_size"]
    if [image.shape[1], image.shape[0]] != source_size:
        raise ValueError("Camera resolution differs from the calibration")
    factor = min(1.0, output_width/image.shape[1])
    size = (round(image.shape[1]*factor), round(image.shape[0]*factor))
    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    matrix = np.asarray(result["camera_matrix"], np.float64).copy()
    rectified = np.asarray(result["rectified_matrix"], np.float64).copy()
    matrix[:2] *= factor; rectified[:2] *= factor
    distortion = np.asarray(result["distortion"], np.float64)
    key = (result["model"], size, matrix.tobytes(), distortion.tobytes(), rectified.tobytes())
    maps = _REMAP_CACHE.get(key)
    if maps is None:
        if result["model"] == "fisheye":
            maps = cv2.fisheye.initUndistortRectifyMap(matrix, distortion, np.eye(3), rectified, size, cv2.CV_16SC2)
        else:
            maps = cv2.initUndistortRectifyMap(matrix, distortion, None, rectified, size, cv2.CV_16SC2)
        _REMAP_CACHE[key] = maps
        while len(_REMAP_CACHE) > 4:
            _REMAP_CACHE.popitem(last=False)
    return cv2.remap(resized, maps[0], maps[1], cv2.INTER_LINEAR)
