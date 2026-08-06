import cv2
import json
import math
import pickle
import re
import numpy as np
from pathlib import Path


def _has_cuda():
    try:
        return cv2.cuda.getCudaEnabledDeviceCount() > 0
    except Exception:
        return False


USE_CUDA = _has_cuda()


def read_dji_xmp(path):
    with open(path, "rb") as f:
        raw = f.read(131072)
    xmp_start = raw.find(b"<x:xmpmeta")
    if xmp_start < 0:
        with open(path, "rb") as f:
            raw = f.read()
        xmp_start = raw.find(b"<x:xmpmeta")
    if xmp_start < 0:
        return None
    xmp_end = raw.find(b"</x:xmpmeta", xmp_start)
    if xmp_end < 0:
        return None
    xmp = raw[xmp_start : xmp_end + 12].decode("utf-8", errors="replace")

    fields = {}
    for key in ["GimbalYawDegree", "GimbalPitchDegree", "GimbalRollDegree"]:
        m = re.search(rf'drone-dji:{key}="([^"]+)"', xmp)
        if m:
            fields[key] = float(m.group(1))
    return fields if "GimbalYawDegree" in fields and "GimbalPitchDegree" in fields else None


def get_camera_fov(path):
    from PIL import Image
    from PIL.ExifTags import TAGS

    img = Image.open(path)
    exif = img._getexif()
    if not exif:
        return None, None

    focal_35mm = None
    for tag_id, value in exif.items():
        tag = TAGS.get(tag_id, tag_id)
        if tag == "FocalLengthIn35mmFilm":
            focal_35mm = float(value)

    w, h = img.size
    if focal_35mm and focal_35mm > 0:
        diag_35mm = math.sqrt(36**2 + 24**2)
        diag_fov = 2 * math.atan(diag_35mm / (2 * focal_35mm))
        diag_px = math.sqrt(w**2 + h**2)
        h_fov = 2 * math.atan(math.tan(diag_fov / 2) * w / diag_px)
        v_fov = 2 * math.atan(math.tan(diag_fov / 2) * h / diag_px)
        return h_fov, v_fov
    return None, None


def has_dji_metadata(image_paths):
    for p in image_paths[:3]:
        if read_dji_xmp(str(p)):
            return True
    return False


def rotation_matrix(yaw_rad, pitch_rad, roll_rad=0.0):
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)
    R_yaw = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    R_pitch = np.array([[1, 0, 0], [0, cp, sp], [0, -sp, cp]])
    R_roll = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return R_yaw @ R_pitch @ R_roll


def angular_distance(yaw1, pitch1, yaw2, pitch2):
    r1 = math.radians(yaw1), math.radians(pitch1)
    r2 = math.radians(yaw2), math.radians(pitch2)
    x1 = math.cos(r1[1]) * math.cos(r1[0])
    y1 = math.sin(r1[1])
    z1 = math.cos(r1[1]) * math.sin(r1[0])
    x2 = math.cos(r2[1]) * math.cos(r2[0])
    y2 = math.sin(r2[1])
    z2 = math.cos(r2[1]) * math.sin(r2[0])
    dot = max(-1.0, min(1.0, x1 * x2 + y1 * y2 + z1 * z2))
    return math.degrees(math.acos(dot))


def _wrap_angle(deg):
    return ((deg + 180) % 360) - 180


def _match_offsets(matches, kp_i, kp_j, R_i, R_j, fx, fy, cx, cy, sift_scale):
    """Offsets between matched points under the given rotations, in degrees.

    Returns the median lon/lat offsets (which drive the yaw/pitch corrections),
    plus the median and median-absolute-deviation of the true angular separation
    between the matched directions. The angular separation is used for vetting
    because lon/lat offsets blow up near the poles, so a steeply pitched pair
    would otherwise look far worse than it is."""
    offsets_yaw = []
    offsets_pitch = []
    offsets_ang = []
    for m in matches:
        pt_i = kp_i[m.queryIdx].pt
        pt_j = kp_j[m.trainIdx].pt

        dir_i_cam = np.array([
            (pt_i[0] * sift_scale - cx) / fx,
            -(pt_i[1] * sift_scale - cy) / fy,
            1.0,
        ])
        dir_i_cam /= np.linalg.norm(dir_i_cam)
        dir_i_world = R_i @ dir_i_cam
        lon_i = math.atan2(dir_i_world[0], dir_i_world[2])
        lat_i = math.asin(np.clip(dir_i_world[1], -1, 1))

        dir_j_cam = np.array([
            (pt_j[0] * sift_scale - cx) / fx,
            -(pt_j[1] * sift_scale - cy) / fy,
            1.0,
        ])
        dir_j_cam /= np.linalg.norm(dir_j_cam)
        dir_j_world = R_j @ dir_j_cam
        lon_j = math.atan2(dir_j_world[0], dir_j_world[2])
        lat_j = math.asin(np.clip(dir_j_world[1], -1, 1))

        d_lon = lon_i - lon_j
        if d_lon > math.pi:
            d_lon -= 2 * math.pi
        elif d_lon < -math.pi:
            d_lon += 2 * math.pi

        offsets_yaw.append(math.degrees(d_lon))
        offsets_pitch.append(math.degrees(lat_i - lat_j))
        offsets_ang.append(math.degrees(
            math.acos(float(np.clip(np.dot(dir_i_world, dir_j_world), -1, 1)))))

    offsets_yaw = np.array(offsets_yaw)
    offsets_pitch = np.array(offsets_pitch)
    offsets_ang = np.array(offsets_ang)
    med_yaw = float(np.median(offsets_yaw))
    med_pitch = float(np.median(offsets_pitch))
    med_ang = float(np.median(offsets_ang))
    mad_ang = float(np.median(np.abs(offsets_ang - med_ang)))
    return med_yaw, med_pitch, med_ang, mad_ang


# Gimbal metadata is accurate to ~12 deg at worst, so anything wildly disagreeing
# with it is a mismatch rather than a correction worth making. Across the sample
# sets, genuine pairs stay under 12 deg offset / 1.4 deg scatter while a bogus
# water match sat at 65 deg / 6.8 deg, so these thresholds separate them cleanly.
MAX_METADATA_OFFSET = 20.0   # deg: reject a pair implying a larger correction
MAX_MATCH_SCATTER = 2.5      # deg: reject a pair whose matches disagree this much

# Pull toward the gimbal metadata. The pair terms only constrain *relative*
# orientations, so without this the whole solution is free to rotate as a body and
# the horizon ends up tilted. Weighted per-image against ~4800 match residuals, so
# it fixes the gauge without meaningfully fighting the data.
METADATA_REG_WEIGHT = 1.0
# Huber transition, in units of unit-vector difference (0.01 ~ 0.57 deg). Matches
# worse than this are downweighted rather than allowed to drag the solution.
BA_ROBUST_SCALE = 0.01
# EXIF FocalLengthIn35mmFilm is a rounded integer and on this camera implies an
# hFOV ~4.5 deg too wide, so the focal length is fitted. These bracket the fit:
# outside them, believe the EXIF rather than a pathological solve.
FOCAL_SCALE_BOUNDS = (0.8, 1.25)


def refine_rotations(entries, h_fov, v_fov, img_w, img_h):
    """Refine gimbal rotations and the focal length by bundle adjustment.

    Returns (entries, fx, fy). The focal length is an output, not an input: the
    EXIF value is only a starting guess and is usually wrong by enough to dominate
    the residual misregistration."""
    fx = img_w / (2 * math.tan(h_fov / 2))
    fy = img_h / (2 * math.tan(v_fov / 2))
    cx, cy = img_w / 2.0, img_h / 2.0

    for e in entries:
        e["yaw"] = _wrap_angle(e["yaw"])

    # Metadata angles, kept as the anchor the optimization is regularized towards.
    yaw0 = [e["yaw"] for e in entries]
    pitch0 = [e["pitch"] for e in entries]

    sift = cv2.SIFT_create(nfeatures=2000)
    FLANN_INDEX_KDTREE = 1
    flann = cv2.FlannBasedMatcher(
        dict(algorithm=FLANN_INDEX_KDTREE, trees=5), dict(checks=50)
    )

    sift_scale = 2
    all_kp, all_desc = [], []
    for entry in entries:
        gray = cv2.cvtColor(entry["img"], cv2.COLOR_BGR2GRAY)
        small = cv2.resize(gray, (gray.shape[1] // sift_scale, gray.shape[0] // sift_scale))
        kp, desc = sift.detectAndCompute(small, None)
        all_kp.append(kp)
        all_desc.append(desc)

    h_fov_deg = math.degrees(h_fov)
    v_fov_deg = math.degrees(v_fov)
    max_overlap_dist = max(h_fov_deg, v_fov_deg) * 0.95
    n = len(entries)

    pair_indices = []
    for i in range(n):
        for j in range(i + 1, n):
            dist = angular_distance(
                entries[i]["yaw"], entries[i]["pitch"],
                entries[j]["yaw"], entries[j]["pitch"],
            )
            if dist < max_overlap_dist:
                pair_indices.append((i, j))

    pair_inlier_matches = {}
    for i, j in pair_indices:
        if all_desc[i] is None or all_desc[j] is None:
            continue
        if len(all_kp[i]) < 20 or len(all_kp[j]) < 20:
            continue

        raw = flann.knnMatch(all_desc[i], all_desc[j], k=2)
        good = []
        for pair in raw:
            if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance:
                good.append(pair[0])

        if len(good) < 15:
            continue

        pts1 = np.float32([all_kp[i][m.queryIdx].pt for m in good])
        pts2 = np.float32([all_kp[j][m.trainIdx].pt for m in good])
        _, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 4.0)
        if mask is None:
            continue

        inlier_matches = [m for m, flag in zip(good, mask.ravel()) if flag]
        if len(inlier_matches) >= 10:
            pair_inlier_matches[(i, j)] = inlier_matches

    print(f"    {len(pair_inlier_matches)} pairs with RANSAC-filtered matches.")

    # Vet each pair against the metadata rotations. RANSAC only guarantees the
    # matches are consistent with *a* homography — on water or foliage that can be
    # a completely wrong pairing, and a single such pair drags an image tens of
    # degrees out of place.
    Rs_meta = [
        rotation_matrix(math.radians(e["yaw"]), math.radians(e["pitch"]),
                        math.radians(e["roll"]))
        for e in entries
    ]
    vetted = {}
    for (i, j), matches in pair_inlier_matches.items():
        _, _, med_ang, mad_ang = _match_offsets(
            matches, all_kp[i], all_kp[j], Rs_meta[i], Rs_meta[j],
            fx, fy, cx, cy, sift_scale)

        if med_ang > MAX_METADATA_OFFSET or mad_ang > MAX_MATCH_SCATTER:
            print(f"    Rejecting pair {i}-{j} ({len(matches)} matches): "
                  f"{med_ang:.1f} deg from metadata, {mad_ang:.1f} deg scatter")
            continue
        vetted[(i, j)] = matches

    pair_inlier_matches = vetted
    print(f"    {len(pair_inlier_matches)} pairs kept after vetting against metadata.")

    if not pair_inlier_matches:
        print("  No usable pairs — keeping metadata rotations and EXIF focal length.")
        return entries, fx, fy

    # Bundle adjustment over every match.
    #
    # The previous solver reduced each pair to a single median lon/lat offset and
    # fitted two angles per image. That model cannot express a roll, a focal
    # length error, or anything else with structure across the frame — those
    # residuals simply vanish into the median, so it reports convergence while
    # leaving real misregistration behind. On the reference set it settled at
    # 1.13 deg (26 px at 8192 wide). Fitting all matches, with roll and a shared
    # focal length free, reaches 0.05 deg (1.2 px).
    from scipy.optimize import least_squares

    idx_i, idx_j, pts_i, pts_j = [], [], [], []
    for (i, j), matches in pair_inlier_matches.items():
        for m in matches:
            idx_i.append(i)
            idx_j.append(j)
            pts_i.append(all_kp[i][m.queryIdx].pt)
            pts_j.append(all_kp[j][m.trainIdx].pt)
    idx_i = np.array(idx_i)
    idx_j = np.array(idx_j)
    pts_i = np.array(pts_i, dtype=np.float64)
    pts_j = np.array(pts_j, dtype=np.float64)

    def _rotations(yaw, pitch, roll):
        cy_, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll), np.sin(roll)
        m = len(yaw)
        Ry = np.zeros((m, 3, 3)); Rp = np.zeros((m, 3, 3)); Rr = np.zeros((m, 3, 3))
        Ry[:, 0, 0] = cy_; Ry[:, 0, 2] = sy; Ry[:, 1, 1] = 1; Ry[:, 2, 0] = -sy; Ry[:, 2, 2] = cy_
        Rp[:, 0, 0] = 1; Rp[:, 1, 1] = cp; Rp[:, 1, 2] = sp; Rp[:, 2, 1] = -sp; Rp[:, 2, 2] = cp
        Rr[:, 0, 0] = cr; Rr[:, 0, 1] = -sr; Rr[:, 1, 0] = sr; Rr[:, 1, 1] = cr; Rr[:, 2, 2] = 1
        return Ry @ Rp @ Rr

    def _world_dirs(pts, which, Rs, focal_scale):
        u = (pts[:, 0] * sift_scale - cx) / (fx * focal_scale)
        v = -(pts[:, 1] * sift_scale - cy) / (fy * focal_scale)
        d = np.stack([u, v, np.ones_like(u)], axis=1)
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        return np.einsum("nij,nj->ni", Rs[which], d)

    yaw_rad0 = np.radians(yaw0)
    pitch_rad0 = np.radians(pitch0)
    roll_rad0 = np.radians([e["roll"] for e in entries])

    def _residuals(x):
        yaw, pitch, roll = x[0:n], x[n:2 * n], x[2 * n:3 * n]
        focal_scale = x[3 * n]
        Rs = _rotations(yaw, pitch, roll)
        d_i = _world_dirs(pts_i, idx_i, Rs, focal_scale)
        d_j = _world_dirs(pts_j, idx_j, Rs, focal_scale)
        return np.concatenate([
            (d_i - d_j).ravel(),
            METADATA_REG_WEIGHT * (yaw - yaw_rad0),
            METADATA_REG_WEIGHT * (pitch - pitch_rad0),
            METADATA_REG_WEIGHT * (roll - roll_rad0),
        ])

    x0 = np.concatenate([yaw_rad0, pitch_rad0, roll_rad0, [1.0]])
    sol = least_squares(_residuals, x0, loss="huber", f_scale=BA_ROBUST_SCALE,
                        max_nfev=300)

    yaw, pitch, roll = sol.x[0:n], sol.x[n:2 * n], sol.x[2 * n:3 * n]
    focal_scale = float(sol.x[3 * n])

    Rs = _rotations(yaw, pitch, roll)
    err = np.degrees(np.arccos(np.clip(np.sum(
        _world_dirs(pts_i, idx_i, Rs, focal_scale)
        * _world_dirs(pts_j, idx_j, Rs, focal_scale), axis=1), -1, 1)))

    if not (FOCAL_SCALE_BOUNDS[0] <= focal_scale <= FOCAL_SCALE_BOUNDS[1]):
        print(f"    Fitted focal scale {focal_scale:.3f} is out of range — "
              f"keeping the EXIF focal length.")
        focal_scale = 1.0

    for idx in range(n):
        entries[idx]["yaw"] = _wrap_angle(math.degrees(yaw[idx]))
        entries[idx]["pitch"] = float(np.clip(math.degrees(pitch[idx]), -90, 90))
        entries[idx]["roll"] = math.degrees(roll[idx])

    fx *= focal_scale
    fy *= focal_scale
    fitted_h_fov = math.degrees(2 * math.atan(img_w / (2 * fx)))
    moved = [angular_distance(yaw0[k], pitch0[k], entries[k]["yaw"], entries[k]["pitch"])
             for k in range(n)]
    print(f"  Bundle adjustment over {len(pts_i)} matches in "
          f"{len(pair_inlier_matches)} pairs:")
    print(f"    Residual: median {np.median(err):.3f} deg, 90th pct "
          f"{np.percentile(err, 90):.3f} deg")
    print(f"    Focal: hFOV {math.degrees(h_fov):.2f} -> {fitted_h_fov:.2f} deg "
          f"(scale {focal_scale:.4f})")
    print(f"    Moved from metadata: median {np.median(moved):.2f} deg, "
          f"max {max(moved):.2f} deg; |roll| max "
          f"{np.abs(np.degrees(roll)).max():.2f} deg")
    return entries, fx, fy


def _project_lowres(entries, rotations, fx, fy, cx, cy_img, img_w, img_h,
                    out_width, out_height, scale=4):
    """Project all images to equirectangular at reduced resolution.
    Pre-computes shared direction vectors once for all images."""
    sw, sh = out_width // scale, out_height // scale
    n = len(entries)

    gx, gy = np.meshgrid(
        np.arange(sw, dtype=np.float32),
        np.arange(sh, dtype=np.float32),
    )
    lon = (gx / sw - 0.5) * (2 * np.pi)
    lat = (0.5 - gy / sh) * np.pi
    cos_lat = np.cos(lat)
    dx = cos_lat * np.sin(lon)
    dy = np.sin(lat)
    dz = cos_lat * np.cos(lon)

    small_layers = []
    small_masks = []
    for i in range(n):
        Rt = rotations[i]
        cam_x = Rt[0, 0] * dx + Rt[0, 1] * dy + Rt[0, 2] * dz
        cam_y = Rt[1, 0] * dx + Rt[1, 1] * dy + Rt[1, 2] * dz
        cam_z = Rt[2, 0] * dx + Rt[2, 1] * dy + Rt[2, 2] * dz

        valid = cam_z > 0.01
        safe_z = np.where(valid, cam_z, 1.0)
        px = fx * (cam_x / safe_z) + cx
        py = fy * (-cam_y / safe_z) + cy_img
        in_bounds = valid & (px >= 0) & (px < img_w - 1) & (py >= 0) & (py < img_h - 1)

        map_x = np.where(in_bounds, px, 0).astype(np.float32)
        map_y = np.where(in_bounds, py, 0).astype(np.float32)
        sampled = cv2.remap(entries[i]["img"], map_x, map_y, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)
        for c in range(3):
            sampled[:, :, c] = np.where(in_bounds, sampled[:, :, c], 0)
        small_layers.append(sampled)
        small_masks.append(in_bounds.astype(np.uint8) * 255)

    return small_layers, small_masks, sw, sh


def _find_seams(small_masks, sw, sh, n):
    """Assign each pixel to the image whose coverage center is closest (Voronoi).
    Handles horizontal wrapping for equirectangular projections."""
    pad = sw // 4
    labels = np.full((sh, sw), -1, dtype=np.int16)
    best_d = np.full((sh, sw), -1.0, dtype=np.float32)
    for i in range(n):
        mask = small_masks[i]
        padded = np.concatenate([mask[:, -pad:], mask, mask[:, :pad]], axis=1)
        dist_padded = cv2.distanceTransform(padded, cv2.DIST_L2, 5).astype(np.float32)
        dist = dist_padded[:, pad:pad + sw]
        closer = dist > best_d
        labels = np.where(closer, i, labels)
        best_d = np.where(closer, dist, best_d)
    assigned = np.count_nonzero(labels >= 0)
    print(f"    Seams assigned for {n} images ({assigned} pixels covered).")
    return labels


def _find_seams_from_reference(reference_path, small_layers, small_masks, sw, sh, n):
    """Determine seam labels by matching each pixel to the source image
    that best matches a reference panorama using normalized cross-correlation
    (invariant to exposure differences)."""
    ref = cv2.imread(str(reference_path))
    if ref is None:
        print(f"    Warning: Could not read reference {reference_path}")
        return None
    ref = cv2.resize(ref, (sw, sh), interpolation=cv2.INTER_AREA)
    ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)

    win = 51
    k = (win, win)
    mu_ref = cv2.GaussianBlur(ref_gray, k, 0)
    var_ref = np.maximum(cv2.GaussianBlur(ref_gray ** 2, k, 0) - mu_ref ** 2, 0)

    labels = np.full((sh, sw), -1, dtype=np.int16)
    best_score = np.full((sh, sw), -np.inf, dtype=np.float32)

    for i in range(n):
        mask = small_masks[i] > 0
        layer_gray = cv2.cvtColor(small_layers[i], cv2.COLOR_BGR2GRAY).astype(np.float32)

        mu_layer = cv2.GaussianBlur(layer_gray, k, 0)
        var_layer = np.maximum(cv2.GaussianBlur(layer_gray ** 2, k, 0) - mu_layer ** 2, 0)
        cov = cv2.GaussianBlur(ref_gray * layer_gray, k, 0) - mu_ref * mu_layer

        denom = np.sqrt(var_ref * var_layer) + 1e-10
        ncc = cov / denom

        low_texture = (var_ref < 50) | (var_layer < 50)
        ncc = np.where(low_texture, 0.0, ncc)

        dist = cv2.distanceTransform(small_masks[i], cv2.DIST_L2, 5).astype(np.float32)
        score = ncc + dist * 0.001

        better = mask & (score > best_score)
        labels[better] = i
        best_score[better] = score[better]

    valid = labels >= 0
    labels_u8 = np.clip(labels, 0, 255).astype(np.uint8)
    labels_smooth = cv2.medianBlur(labels_u8, 31).astype(np.int16)
    labels = np.where(valid, labels_smooth, labels)

    assigned = np.count_nonzero(labels >= 0)
    print(f"    Reference-matched seams for {n} images ({assigned} pixels covered).")
    return labels


def _detect_people(entries, fx, fy, cx, cy, out_width, out_height):
    """Detect people in source images and project to equirectangular coordinates.
    Uses multiple cascades (face + upper body) for better coverage.
    Returns list of (image_idx, eq_x, eq_y, eq_w, eq_h) for each detection."""
    cascades = [
        ("upperbody", cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_upperbody.xml"),
         dict(scaleFactor=1.1, minNeighbors=4, minSize=(25, 30))),
        ("frontalface", cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_alt2.xml"),
         dict(scaleFactor=1.1, minNeighbors=3, minSize=(15, 15))),
        ("profileface", cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_profileface.xml"),
         dict(scaleFactor=1.1, minNeighbors=3, minSize=(15, 15))),
    ]

    det_scale = 2
    raw_boxes = []
    img_h_src = entries[0]["img"].shape[0]
    for i, e in enumerate(entries):
        gray = cv2.cvtColor(e["img"], cv2.COLOR_BGR2GRAY)
        small_gray = cv2.resize(gray, (gray.shape[1] // det_scale, gray.shape[0] // det_scale))
        R = rotation_matrix(
            math.radians(e["yaw"]), math.radians(e["pitch"]), math.radians(e["roll"]))

        for cname, cascade, params in cascades:
            scaled_params = dict(params)
            ms = scaled_params["minSize"]
            scaled_params["minSize"] = (max(1, ms[0] // det_scale), max(1, ms[1] // det_scale))
            rects = cascade.detectMultiScale(small_gray, **scaled_params)
            for (x, y, w, h) in rects:
                x, y, w, h = x * det_scale, y * det_scale, w * det_scale, h * det_scale
                if y + h / 2 < img_h_src * 0.25:
                    continue

                if "face" in cname:
                    body_x = x - w
                    body_y = y
                    body_w = w * 3
                    body_h = h * 7
                else:
                    body_x = x
                    body_y = y
                    body_w = w
                    body_h = h

                body_x = max(0, body_x)
                body_y = max(0, body_y)

                corners = [(body_x, body_y), (body_x + body_w, body_y),
                           (body_x, body_y + body_h), (body_x + body_w, body_y + body_h)]
                eq_corners = []
                for px, py in corners:
                    d = np.array([(px - cx) / fx, -(py - cy) / fy, 1.0])
                    d /= np.linalg.norm(d)
                    dw = R @ d
                    lon = math.atan2(dw[0], dw[2])
                    lat = math.asin(np.clip(dw[1], -1, 1))
                    eq_corners.append((
                        (lon / (2 * math.pi) + 0.5) * out_width,
                        (0.5 - lat / math.pi) * out_height,
                    ))
                xs = [c[0] for c in eq_corners]
                ys = [c[1] for c in eq_corners]
                eq_x = min(xs)
                eq_y = min(ys)
                eq_w = max(xs) - eq_x
                eq_h = max(ys) - eq_y

                if "face" not in cname:
                    eq_h *= 1.8
                    eq_w *= 1.3
                    eq_x -= eq_w * 0.15

                raw_boxes.append((i, eq_x, eq_y, eq_w, eq_h))

    # Merge overlapping boxes from same source image (NMS-style)
    detections = []
    used = set()
    for a in range(len(raw_boxes)):
        if a in used:
            continue
        ai, ax, ay, aw, ah = raw_boxes[a]
        merged_x1, merged_y1 = ax, ay
        merged_x2, merged_y2 = ax + aw, ay + ah
        used.add(a)
        for b in range(a + 1, len(raw_boxes)):
            if b in used:
                continue
            bi, bx, by, bw, bh = raw_boxes[b]
            if bi != ai:
                continue
            # Check overlap
            ox1 = max(ax, bx)
            oy1 = max(ay, by)
            ox2 = min(ax + aw, bx + bw)
            oy2 = min(ay + ah, by + bh)
            if ox1 < ox2 and oy1 < oy2:
                merged_x1 = min(merged_x1, bx)
                merged_y1 = min(merged_y1, by)
                merged_x2 = max(merged_x2, bx + bw)
                merged_y2 = max(merged_y2, by + bh)
                used.add(b)
        detections.append((ai, merged_x1, merged_y1, merged_x2 - merged_x1, merged_y2 - merged_y1))

    print(f"    {len(raw_boxes)} raw detections -> {len(detections)} merged across {len(entries)} images.")
    return detections


def _preserve_subjects(seam_labels, small_layers, small_masks, detections, n,
                       out_width, out_height,
                       crop_x0=0, crop_y0=0, crop_sw=None, crop_sh=None,
                       full_sw=None, full_sh=None):
    """Override seam labels so each detected person comes from one sharp source."""
    sh, sw = seam_labels.shape
    if full_sw is None:
        full_sw = sw
    if full_sh is None:
        full_sh = sh
    scale_x = full_sw / out_width
    scale_y = full_sh / out_height

    # Cluster detections by proximity
    centers = [(d[1] + d[3] / 2, d[2] + d[4] / 2) for d in detections]
    cluster_radius = 150
    used = set()
    clusters = []
    for idx in range(len(detections)):
        if idx in used:
            continue
        cluster = [idx]
        used.add(idx)
        for other in range(idx + 1, len(detections)):
            if other in used:
                continue
            dist = math.sqrt((centers[idx][0] - centers[other][0]) ** 2 +
                             (centers[idx][1] - centers[other][1]) ** 2)
            if dist < cluster_radius:
                cluster.append(other)
                used.add(other)
        clusters.append(cluster)

    # Pre-compute sharpness for each image at low res
    sharpness = []
    for i in range(n):
        gray = cv2.cvtColor(small_layers[i], cv2.COLOR_BGR2GRAY)
        lap = cv2.Laplacian(gray, cv2.CV_32F)
        sharpness.append(cv2.GaussianBlur(lap ** 2, (11, 11), 0))

    # Pre-compute distance from edge for each image
    dist_maps = []
    for i in range(n):
        dist_maps.append(
            cv2.distanceTransform(small_masks[i], cv2.DIST_L2, 5).astype(np.float32))

    fixed = 0
    for cluster in clusters:
        # Compute merged bounding box in equirectangular
        all_x = [detections[idx][1] for idx in cluster]
        all_y = [detections[idx][2] for idx in cluster]
        all_r = [detections[idx][1] + detections[idx][3] for idx in cluster]
        all_b = [detections[idx][2] + detections[idx][4] for idx in cluster]
        eq_x1 = min(all_x)
        eq_y1 = min(all_y)
        eq_x2 = max(all_r)
        eq_y2 = max(all_b)

        pad = 40
        lx1 = max(0, int(eq_x1 * scale_x) - crop_x0 - pad)
        ly1 = max(0, int(eq_y1 * scale_y) - crop_y0 - pad)
        lx2 = min(sw, int(eq_x2 * scale_x) - crop_x0 + pad)
        ly2 = min(sh, int(eq_y2 * scale_y) - crop_y0 + pad)
        if lx2 <= lx1 or ly2 <= ly1:
            continue

        # Find the best source image for this region
        source_images = set(int(detections[idx][0]) for idx in cluster)
        best_score = -1
        best_img = -1

        for img_i in source_images:
            region_mask = small_masks[img_i][ly1:ly2, lx1:lx2] > 0
            coverage = np.count_nonzero(region_mask)
            if coverage < (ly2 - ly1) * (lx2 - lx1) * 0.5:
                continue
            sharp = float(np.mean(sharpness[img_i][ly1:ly2, lx1:lx2][region_mask]))
            dist = float(np.mean(dist_maps[img_i][ly1:ly2, lx1:lx2][region_mask]))
            score = sharp + dist * 0.5
            if score > best_score:
                best_score = score
                best_img = img_i

        if best_img < 0:
            continue

        region_valid = small_masks[best_img][ly1:ly2, lx1:lx2] > 0
        seam_labels[ly1:ly2, lx1:lx2] = np.where(
            region_valid, best_img, seam_labels[ly1:ly2, lx1:lx2])
        fixed += 1

    print(f"    Preserved {fixed} subjects from {len(clusters)} detections.")
    return seam_labels


# --- Photometry -------------------------------------------------------------
#
# Exposure maths is only meaningful on linear light, but JPEGs are sRGB-encoded
# and a drone auto-exposes every frame of a pano independently (2 EV of spread is
# routine). Scaling the encoded 8-bit values directly — the obvious thing — both
# brightens shadows more than a real exposure change would and destroys every
# highlight that lands past 255. So the pipeline converts to linear, normalises
# each frame by the exposure it was actually shot at, blends in float, and only
# quantises at the very end through a shoulder that has somewhere to put the
# highlights.

def _srgb_to_linear(srgb255):
    """sRGB-encoded values in [0, 255] (float) -> linear light in [0, 1]."""
    x = srgb255.astype(np.float32) * (1.0 / 255.0)
    lin = cv2.pow((x + 0.055) * (1.0 / 1.055), 2.4)
    return np.where(x <= 0.04045, x * (1.0 / 12.92), lin).astype(np.float32)


def _linear_to_srgb(lin):
    """Linear light in [0, 1] -> sRGB-encoded values in [0, 1]."""
    lin = np.clip(lin, 0.0, 1.0).astype(np.float32)
    enc = 1.055 * cv2.pow(lin, 1.0 / 2.4) - 0.055
    return np.where(lin <= 0.0031308, lin * 12.92, enc).astype(np.float32)


# Where the highlight shoulder starts, in linear light. 0.18 is photographic
# mid-grey: everything from black up to mid-grey is rendered with an exactly
# linear response, and only above it does compression begin.
SHOULDER_KNEE = 0.18
# Mid-tone the scene's median is lifted toward. 0.03 linear is sRGB ~48, about
# where a well-exposed landscape sits. The lift is capped at one stop so a scene
# that really is dark stays dark instead of being metered up to grey.
TARGET_MIDTONE = 0.03
MAX_MIDTONE_LIFT = 2.0


def _shoulder(lin, knee=SHOULDER_KNEE):
    """Soft highlight rolloff: identity below the knee, asymptotic to 1.0 above.

        y = 1 - (1-k)^2 / (x - 2k + 1)

    C1-continuous at the knee (slope 1 either side), so there is no visible kink
    where compression starts. The tail is hyperbolic rather than exponential
    deliberately: a sun is a few hundred times brighter than the landscape around
    it, and a tanh-style rolloff saturates to flat white a stop or two past the
    knee, throwing away every cloud near the sun. This one still resolves
    detail at 100x the knee."""
    out = np.clip(lin, 0.0, None).astype(np.float32)
    hi = out > knee
    out[hi] = 1.0 - (1.0 - knee) ** 2 / (out[hi] - 2.0 * knee + 1.0)
    return out


def read_exposure(path):
    """Relative light-gathering of one frame: exposure time * ISO / f-number^2.

    Two frames of the same scene differ in encoded brightness by the ratio of
    these, so dividing by it puts every frame on a common radiance scale."""
    from PIL import Image
    from PIL.ExifTags import TAGS

    exif = Image.open(path)._getexif()
    if not exif:
        return None
    tags = {TAGS.get(k, k): v for k, v in exif.items()}
    try:
        et = float(tags["ExposureTime"])
    except (KeyError, TypeError, ValueError):
        return None
    iso = float(tags.get("ISOSpeedRatings") or 100)
    try:
        f_number = float(tags.get("FNumber") or 0)
    except (TypeError, ValueError):
        f_number = 0
    aperture_term = 1.0 / (f_number ** 2) if f_number > 0 else 1.0
    return et * iso * aperture_term


def compute_exposure_scales(paths):
    """Per-frame multipliers (linear light) that undo per-frame auto-exposure.

    Normalised against the most-exposed frame, so every scale is >= 1 and the
    scene is only ever brightened into the float headroom, never darkened into
    quantisation noise. Returns all-ones if the EXIF is unusable."""
    exposures = [read_exposure(p) for p in paths]
    if any(e is None or e <= 0 for e in exposures):
        missing = sum(1 for e in exposures if e is None or e <= 0)
        print(f"  No usable exposure EXIF in {missing}/{len(paths)} images — "
              f"relying on the overlap solve alone.")
        return np.ones(len(paths))

    exposures = np.array(exposures, dtype=np.float64)
    scales = exposures.max() / exposures
    ev = np.log2(exposures.max() / exposures.min())
    print(f"  EXIF exposure spread: {ev:.2f} EV "
          f"({len(set(np.round(exposures, 9)))} distinct settings), "
          f"scales {scales.min():.2f}-{scales.max():.2f}x")
    return scales


def _rescale_srgb(img_u8, scale):
    """Apply a linear-light gain to an 8-bit sRGB image, rolling off highlights.

    Used for the half-res preview and seam-search layers, which have to stay
    8-bit. The full-res path never round-trips through this."""
    if abs(scale - 1.0) < 0.01:
        return img_u8
    lin = _srgb_to_linear(img_u8.astype(np.float32)) * scale
    return (_linear_to_srgb(_shoulder(lin)) * 255.0).astype(np.uint8)


def _compute_exposure_gains(small_layers, small_masks, n, prescale=None, max_ev=0.5):
    """Solve residual per-image gains from overlap brightness ratios.

    `prescale` is the EXIF normalisation, applied here in linear light rather
    than baked into the layers beforehand: the layers are 8-bit and would have to
    be shoulder-compressed to survive a 4x scale, and compressed highlights would
    read as "too dark" to this solver and provoke a correction that undoes them.
    So this only mops up metering variation the exposure settings don't explain —
    hence the tight clamp."""
    A_rows = []
    b_rows = []

    if prescale is None:
        prescale = np.ones(n)

    linear_lum = []
    saturated = []
    for i, layer in enumerate(small_layers):
        lin = _srgb_to_linear(layer.astype(np.float32))
        lum = (0.0722 * lin[:, :, 0] + 0.7152 * lin[:, :, 1]
               + 0.2126 * lin[:, :, 2])
        # Saturation happened at capture, so it is judged before the prescale.
        saturated.append(lum > 0.98)
        linear_lum.append(lum * prescale[i])

    for i in range(n):
        for j in range(i + 1, n):
            overlap = (small_masks[i] > 0) & (small_masks[j] > 0)
            n_overlap = np.count_nonzero(overlap)
            if n_overlap < 500:
                continue

            gray_i = linear_lum[i]
            gray_j = linear_lum[j]

            # Saturated pixels carry no ratio information — their true value was
            # lost at capture — and would bias the fit toward under-correction.
            usable = overlap & ~saturated[i] & ~saturated[j]
            if np.count_nonzero(usable) < 500:
                continue
            n_overlap = np.count_nonzero(usable)

            mean_i = np.mean(gray_i[usable])
            mean_j = np.mean(gray_j[usable])
            if mean_i < 1e-4 or mean_j < 1e-4:
                continue

            weight = math.sqrt(n_overlap)
            row = np.zeros(n)
            row[i] = weight
            row[j] = -weight
            A_rows.append(row)
            b_rows.append(math.log(mean_j / mean_i) * weight)

    if not A_rows:
        return np.ones(n)

    for i in range(n):
        row = np.zeros(n)
        row[i] = 0.1
        A_rows.append(row)
        b_rows.append(0.0)

    A = np.array(A_rows)
    b = np.array(b_rows)
    log_gains, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    limit = max_ev * math.log(2.0)
    gains = np.exp(np.clip(log_gains, -limit, limit))

    print(f"  Residual gains: {', '.join(f'{g:.2f}' for g in gains)}")
    return gains


# --- Multi-band blending ----------------------------------------------------
#
# A single feathered average can only trade one artefact for another: a narrow
# feather leaves a visible exposure step at the seam, a wide one smears every
# misaligned edge across its whole width. Multi-band blending picks the width per
# spatial frequency — wide for the low frequencies that carry exposure and colour
# differences, narrow for the fine detail that ghosts — so both go at once.

# Each level doubles the scale at which a seam is hidden; 6 levels at 8192 wide
# blends the lowest band over roughly a quarter of the frame.
MULTIBAND_LEVELS = 6


def _gaussian_pyramid(img, n_levels):
    gp = [img]
    for _ in range(n_levels - 1):
        gp.append(cv2.pyrDown(gp[-1]))
    return gp


def _laplacian_pyramid(img, n_levels):
    gp = _gaussian_pyramid(img, n_levels)
    lp = []
    for l in range(n_levels - 1):
        up = cv2.pyrUp(gp[l + 1], dstsize=(gp[l].shape[1], gp[l].shape[0]))
        lp.append(gp[l] - up)
        gp[l] = None      # a full-res level is ~400 MB at 8192x4096; free as we go
    lp.append(gp[-1])
    return lp


def _collapse_pyramid(lp):
    out = lp[-1]
    for l in range(len(lp) - 2, -1, -1):
        out = cv2.pyrUp(out, dstsize=(lp[l].shape[1], lp[l].shape[0])) + lp[l]
    return out


def _fill_invalid(img, valid, n_levels=MULTIBAND_LEVELS):
    """Extrapolate an image smoothly into the region it does not cover.

    Without this, the Laplacian pyramid of a projected frame sees a cliff from
    real pixels straight down to black at the edge of that frame's coverage. The
    coarse bands smear that cliff hundreds of pixels inward — a dark halo around
    every frame. Filling first costs one push-pull pass and removes the cliff.
    Values where `valid` is true are returned unchanged."""
    num = [img * valid[:, :, np.newaxis]]
    den = [valid]
    for _ in range(n_levels - 1):
        num.append(cv2.pyrDown(num[-1]))
        den.append(cv2.pyrDown(den[-1]))

    est = num[-1] / np.maximum(den[-1], 1e-6)[:, :, np.newaxis]
    for l in range(n_levels - 2, -1, -1):
        up = cv2.pyrUp(est, dstsize=(num[l].shape[1], num[l].shape[0]))
        d = np.minimum(den[l], 1.0)[:, :, np.newaxis]
        here = num[l] / np.maximum(d, 1e-6)
        est = here * d + up * (1.0 - d)
    return est


class _BandBlender:
    """Accumulates images into a Laplacian pyramid mosaic, one image at a time.

    Only the pyramids are held, never all the projected layers at once, so the
    cost is set by the output size rather than by the number of images."""

    def __init__(self, height, width, n_levels=MULTIBAND_LEVELS, wrap=True):
        self.h, self.w = height, width
        self.n = n_levels
        # Circular padding so the 360 deg wrap is not treated as an image border,
        # which would otherwise mirror content back across the seam at the coarse
        # levels. Sized to outlast the pyramid: a few pixels at the top level.
        self.pad = (1 << (n_levels - 1)) * 8 if wrap else 0
        self.acc = None
        self.wacc = None
        # Per-pixel envelope of the frames that actually saw each pixel. The coarse
        # bands overshoot it wherever two frames disagree about where an edge is —
        # at 6 levels, on 17% of pixels — which shows up as dark or bright wedges
        # against high-contrast edges. Clamping to the envelope removes that
        # without touching any blend that stays between its inputs.
        self.lo = np.full((height, width, 3), np.inf, dtype=np.float32)
        self.hi = np.full((height, width, 3), -np.inf, dtype=np.float32)

    def _pad(self, a):
        if not self.pad:
            return a
        return np.concatenate([a[:, self.w - self.pad:], a, a[:, :self.pad]], axis=1)

    def add(self, img, valid, weight):
        """Add one projected layer. `img` is zero outside `valid`; the envelope is
        taken from the real pixels, and the pyramid from a filled copy."""
        np.minimum(self.lo, img, out=self.lo, where=valid[:, :, np.newaxis])
        np.maximum(self.hi, img, out=self.hi, where=valid[:, :, np.newaxis])

        lp = _laplacian_pyramid(self._pad(_fill_invalid(img, valid.astype(np.float32))),
                                self.n)
        gw = _gaussian_pyramid(self._pad(weight), self.n)
        if self.acc is None:
            self.acc = [np.zeros_like(band) for band in lp]
            self.wacc = [np.zeros_like(band) for band in gw]
        for l in range(self.n):
            self.acc[l] += lp[l] * gw[l][:, :, np.newaxis]
            self.wacc[l] += gw[l]
            lp[l] = None
            gw[l] = None

    def result(self):
        if self.acc is None:
            return np.zeros((self.h, self.w, 3), dtype=np.float32)
        for l in range(self.n):
            self.acc[l] /= np.maximum(self.wacc[l], 1e-6)[:, :, np.newaxis]
        self.wacc = None
        out = _collapse_pyramid(self.acc)
        self.acc = None
        if self.pad:
            out = out[:, self.pad:self.pad + self.w]

        never_seen = ~np.isfinite(self.lo)
        self.lo[never_seen] = 0.0
        self.hi[~np.isfinite(self.hi)] = 0.0
        return np.clip(out, self.lo, self.hi)


def _project_full(img, Rt, fx, fy, cx, cy_img, img_w, img_h,
                  full_w, full_h, x0=0, x1=None, y0=0, y1=None,
                  gpu_img=None, strip=512):
    """Project one image over a region of the equirectangular output.

    Done a strip at a time so the large intermediates stay strip-sized rather
    than full-frame — at 8192x4096 that is the difference between a few hundred
    MB and a few GB. `full_w`/`full_h` describe the whole sphere; x0..x1, y0..y1
    select the region actually being rendered, which the editor may have cropped."""
    if x1 is None:
        x1 = full_w
    if y1 is None:
        y1 = full_h
    h, w = y1 - y0, x1 - x0
    projected = np.zeros((h, w, 3), dtype=np.float32)
    valid = np.zeros((h, w), dtype=bool)
    for sy0 in range(y0, y1, strip):
        sy1 = min(sy0 + strip, y1)
        directions = _compute_strip_directions(full_w, full_h, sy0, sy1, x0=x0, x1=x1)
        if gpu_img is not None:
            p, v = _project_strip_cuda(gpu_img, Rt, fx, fy, cx, cy_img,
                                       img_w, img_h, directions)
        else:
            p, v = _project_strip(img, Rt, fx, fy, cx, cy_img, img_w, img_h, directions)
        projected[sy0 - y0:sy1 - y0] = p
        valid[sy0 - y0:sy1 - y0] = v
    return projected, valid


def _tonemap(lin, mask, stats_mask=None):
    """Linear radiance map -> 8-bit sRGB, with highlights rolled off not clipped.

    `stats_mask` selects the pixels the exposure is metered from; pass the real
    coverage so a synthesised zenith — a quarter of the frame, and bright sky —
    cannot drag the metering off the photographed part of the scene."""
    lum = 0.0722 * lin[:, :, 0] + 0.7152 * lin[:, :, 1] + 0.2126 * lin[:, :, 2]
    meter = stats_mask if stats_mask is not None else mask
    sample = lum[meter] if meter is not None else lum.ravel()
    sample = sample[sample > 0]

    gain = 1.0
    if sample.size:
        mid = float(np.median(sample))
        # Anchored on the median, never on a high percentile: the sun is a few
        # hundred times brighter than the landscape, so anchoring on highlights
        # drags the whole scene into the dark to make room for something that is
        # supposed to blow out. The shoulder is what handles the sun.
        if mid > 1e-6:
            gain = float(np.clip(TARGET_MIDTONE / mid, 1.0, MAX_MIDTONE_LIFT))
        print(f"  Tone map: median luminance {mid:.4f} linear, "
              f"global gain {gain:.3f}, knee {SHOULDER_KNEE}")

    out = _shoulder(lin * gain)
    out = (_linear_to_srgb(out) * 255.0).astype(np.uint8)
    if mask is not None:
        out[~mask] = 0
    return out


def _blur_row_wrap(row, sigma):
    """Blur one equirectangular row horizontally, wrapping at the seam."""
    if sigma < 0.3:
        return row
    n = row.shape[0]
    pad = min(n, int(3 * sigma) + 1)
    ext = np.concatenate([row[n - pad:], row, row[:pad]], axis=0)
    k = 2 * int(3 * sigma) + 1
    # ksize (k, 1) keeps the blur strictly horizontal on this 1-row image.
    blurred = cv2.GaussianBlur(ext.reshape(1, -1, 3), (k, 1), sigma)
    return blurred[0, pad:pad + n]


def _fill_pole(lin, mask, small_w=512):
    """Synthesise the zenith cap the camera never photographed.

    A DJI sphere pano tops out around +43 deg elevation — the top row of shots
    plus half a vertical FOV — leaving a quarter of the equirectangular frame
    empty. There is no data to recover, so the sky is extended upward: each
    missing row is the row below it, blurred horizontally by an amount that grows
    with latitude, so real detail dissolves into a flat cap at the pole exactly
    as the horizontal scale collapses there.

    Synthesised at low resolution (the cap has no detail to preserve) and feathered
    into the real pixels. Returns (filled, coverage) with coverage extended."""
    h, w = mask.shape
    rows_any = np.where(mask.any(axis=1))[0]
    if len(rows_any) == 0 or rows_any[0] == 0:
        return lin, mask

    sw = min(small_w, w)
    sh = max(16, int(round(h * sw / w)))
    m = mask.astype(np.float32)
    num = cv2.resize(lin * m[:, :, np.newaxis], (sw, sh), interpolation=cv2.INTER_AREA)
    den = cv2.resize(m, (sw, sh), interpolation=cv2.INTER_AREA)
    small = num / np.maximum(den, 1e-6)[:, :, np.newaxis]
    solid = den > 0.5

    full_rows = np.where(solid.all(axis=1))[0]
    if len(full_rows) == 0 or full_rows[0] == 0:
        return lin, mask
    top = int(full_rows[0])

    out_small = small.copy()
    for r in range(top - 1, -1, -1):
        lat = math.radians((0.5 - (r + 0.5) / sh) * 180.0)
        # A fixed great-circle blur width costs more pixels the nearer the pole,
        # because the rows themselves are shrinking toward a point.
        sigma = min(sw / 3.0, 1.5 / max(math.cos(lat), 1e-3))
        blurred = _blur_row_wrap(out_small[r + 1], sigma)
        keep = solid[r][:, np.newaxis]
        out_small[r] = np.where(keep, small[r], blurred)

    cap = cv2.resize(out_small, (w, h), interpolation=cv2.INTER_LINEAR)
    filled = np.where(mask[:, :, np.newaxis], lin, cap)

    # Everything above the first fully-covered row now has data, real or synthetic.
    top_full = min(h - 1, int(round(top * h / sh)) + 1)

    # Feather across the coverage boundary so the sharp real sky does not meet
    # the smooth synthetic one at a hard edge. Confined to the cap: elsewhere —
    # the nadir hole in particular — real pixels must stay untouched.
    feather = max(2.0, h / 400.0)
    alpha = cv2.GaussianBlur(m, (0, 0), feather)
    alpha[min(h, top_full + int(4 * feather)):] = 1.0
    alpha = alpha[:, :, np.newaxis]
    filled = filled * alpha + cap * (1.0 - alpha)
    coverage = mask.copy()
    coverage[:top_full] = True
    print(f"  Zenith fill: synthesised {rows_any[0]} rows above "
          f"{90 - rows_any[0] / h * 180:.1f} deg")
    return filled.astype(np.float32), coverage


def _compute_strip_directions(out_width, out_height, y0, y1, x0=0, x1=None):
    """Pre-compute world direction vectors for a strip. Reusable across images."""
    if x1 is None:
        x1 = out_width
    gx, gy = np.meshgrid(
        np.arange(x0, x1, dtype=np.float32),
        np.arange(y0, y1, dtype=np.float32),
    )
    lon = (gx / out_width - 0.5) * (2 * np.pi)
    lat = (0.5 - gy / out_height) * np.pi
    cos_lat = np.cos(lat)
    return (cos_lat * np.sin(lon), np.sin(lat), cos_lat * np.cos(lon))


def _project_strip(img, Rt, fx, fy, cx, cy_img, img_w, img_h, directions):
    """Project one image using pre-computed direction vectors (CPU path)."""
    dx, dy, dz = directions
    h, w = dx.shape

    cam_z = Rt[2, 0] * dx + Rt[2, 1] * dy + Rt[2, 2] * dz
    front = cam_z > 0.01
    cols = np.any(front, axis=0)
    if not np.any(cols):
        return np.zeros((h, w, 3), dtype=np.float32), np.zeros((h, w), dtype=bool)

    x0 = int(np.argmax(cols))
    x1 = int(len(cols) - np.argmax(cols[::-1]))

    sdx = dx[:, x0:x1]
    sdy = dy[:, x0:x1]
    sdz = dz[:, x0:x1]

    cam_x = Rt[0, 0] * sdx + Rt[0, 1] * sdy + Rt[0, 2] * sdz
    cam_y = Rt[1, 0] * sdx + Rt[1, 1] * sdy + Rt[1, 2] * sdz
    cam_z_s = Rt[2, 0] * sdx + Rt[2, 1] * sdy + Rt[2, 2] * sdz

    valid = cam_z_s > 0.01
    safe_z = np.where(valid, cam_z_s, 1.0)

    px = fx * (cam_x / safe_z) + cx
    py = fy * (-cam_y / safe_z) + cy_img

    in_bounds_sub = valid & (px >= 0) & (px < img_w - 1) & (py >= 0) & (py < img_h - 1)

    map_x = np.where(in_bounds_sub, px, 0).astype(np.float32)
    map_y = np.where(in_bounds_sub, py, 0).astype(np.float32)

    sub_sampled = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)
    for c in range(3):
        sub_sampled[:, :, c] = np.where(in_bounds_sub, sub_sampled[:, :, c], 0)

    sampled = np.zeros((h, w, 3), dtype=np.float32)
    sampled[:, x0:x1] = sub_sampled.astype(np.float32)
    in_bounds = np.zeros((h, w), dtype=bool)
    in_bounds[:, x0:x1] = in_bounds_sub
    return sampled, in_bounds


def _project_strip_cuda(gpu_img, Rt, fx, fy, cx, cy_img, img_w, img_h, directions):
    """Project one image using CUDA-accelerated remap."""
    dx, dy, dz = directions
    h, w = dx.shape

    cam_z = Rt[2, 0] * dx + Rt[2, 1] * dy + Rt[2, 2] * dz
    front = cam_z > 0.01
    cols = np.any(front, axis=0)
    if not np.any(cols):
        return np.zeros((h, w, 3), dtype=np.float32), np.zeros((h, w), dtype=bool)

    x0 = int(np.argmax(cols))
    x1 = int(len(cols) - np.argmax(cols[::-1]))

    sdx, sdy, sdz = dx[:, x0:x1], dy[:, x0:x1], dz[:, x0:x1]
    cam_x = Rt[0, 0] * sdx + Rt[0, 1] * sdy + Rt[0, 2] * sdz
    cam_y = Rt[1, 0] * sdx + Rt[1, 1] * sdy + Rt[1, 2] * sdz
    cam_z_s = Rt[2, 0] * sdx + Rt[2, 1] * sdy + Rt[2, 2] * sdz

    valid = cam_z_s > 0.01
    safe_z = np.where(valid, cam_z_s, 1.0)
    px = fx * (cam_x / safe_z) + cx
    py = fy * (-cam_y / safe_z) + cy_img
    in_bounds_sub = valid & (px >= 0) & (px < img_w - 1) & (py >= 0) & (py < img_h - 1)

    map_x = np.where(in_bounds_sub, px, 0).astype(np.float32)
    map_y = np.where(in_bounds_sub, py, 0).astype(np.float32)

    gpu_map_x = cv2.cuda_GpuMat(map_x)
    gpu_map_y = cv2.cuda_GpuMat(map_y)
    gpu_out = cv2.cuda.remap(gpu_img, gpu_map_x, gpu_map_y,
                             interpolation=cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT)
    sub_sampled = gpu_out.download()
    for c in range(3):
        sub_sampled[:, :, c] = np.where(in_bounds_sub, sub_sampled[:, :, c], 0)

    sampled = np.zeros((h, w, 3), dtype=np.float32)
    sampled[:, x0:x1] = sub_sampled.astype(np.float32)
    in_bounds = np.zeros((h, w), dtype=bool)
    in_bounds[:, x0:x1] = in_bounds_sub
    return sampled, in_bounds


def stitch_dji(image_paths, output_path, out_width=8192, out_height=4096, reference_path=None):
    print("DJI metadata detected — using direct projection stitching.")

    entries = []
    for path in image_paths:
        xmp = read_dji_xmp(str(path))
        if xmp is None:
            print(f"  Warning: No DJI metadata in {Path(path).name}, skipping.")
            continue
        img = cv2.imread(str(path))
        if img is None:
            print(f"  Warning: Could not read {Path(path).name}, skipping.")
            continue
        entries.append(
            {
                "path": path,
                "img": img,
                "yaw": xmp["GimbalYawDegree"],
                "pitch": xmp["GimbalPitchDegree"],
                "roll": xmp.get("GimbalRollDegree", 0.0),
            }
        )

    if len(entries) < 2:
        print("Not enough valid DJI images.")
        return None

    h_fov, v_fov = get_camera_fov(str(image_paths[0]))
    if h_fov is None:
        print("Could not determine camera FOV from EXIF.")
        return None

    img_h, img_w = entries[0]["img"].shape[:2]
    print(f"  Camera FOV: {math.degrees(h_fov):.1f} x {math.degrees(v_fov):.1f} deg")
    print(f"  Input: {len(entries)} images at {img_w}x{img_h}")

    print("  Refining rotations (SIFT + bundle adjustment)...")
    # The focal length comes back fitted, not taken from EXIF — see refine_rotations.
    entries, fx, fy = refine_rotations(entries, h_fov, v_fov, img_w, img_h)

    cx, cy_img = img_w / 2.0, img_h / 2.0

    rotations = []
    for e in entries:
        R = rotation_matrix(
            math.radians(e["yaw"]),
            math.radians(e["pitch"]),
            math.radians(e["roll"]),
        )
        rotations.append(R.T)

    print("  Normalising exposure from EXIF...")
    exposure_scales = compute_exposure_scales([e["path"] for e in entries])

    print("  Projecting at half resolution...")
    small_layers, small_masks, sw, sh = _project_lowres(
        entries, rotations, fx, fy, cx, cy_img, img_w, img_h, out_width, out_height, scale=2)

    n = len(entries)
    print("  Solving residual exposure gains from overlaps...")
    gains = _compute_exposure_gains(small_layers, small_masks, n,
                                    prescale=exposure_scales)
    total_scales = exposure_scales * gains

    # Normalise the preview layers once, for the seam search. The full-res path
    # scales in linear light during blending and never round-trips through 8 bits.
    for i in range(n):
        small_layers[i] = _rescale_srgb(small_layers[i], total_scales[i])

    if reference_path:
        print("  Finding seams from reference panorama...")
        seam_labels = _find_seams_from_reference(reference_path, small_layers, small_masks, sw, sh, n)
        if seam_labels is None:
            print("  Falling back to Voronoi seams...")
            seam_labels = _find_seams(small_masks, sw, sh, n)
    else:
        print("  Finding seams...")
        seam_labels = _find_seams(small_masks, sw, sh, n)
        print("  Detecting people...")
        detections = _detect_people(entries, fx, fy, cx, cy_img, out_width, out_height)
        print("  Preserving subjects...")
        seam_labels = _preserve_subjects(seam_labels, small_layers, small_masks, detections, n,
                                         out_width, out_height)

    seam_labels_full = cv2.resize(seam_labels.astype(np.float32),
                                  (out_width, out_height),
                                  interpolation=cv2.INTER_NEAREST).astype(np.int16)

    del small_layers, small_masks

    print(f"  Output: {out_width}x{out_height}")

    print(f"  Projecting and blending ({MULTIBAND_LEVELS}-band)...")
    blender = _BandBlender(out_height, out_width)
    weight_total = np.zeros((out_height, out_width), dtype=np.float32)
    covered = np.zeros((out_height, out_width), dtype=bool)

    for i in range(n):
        region = (seam_labels_full == i)
        if not np.any(region):
            continue

        projected, valid = _project_full(
            entries[i]["img"], rotations[i], fx, fy, cx, cy_img,
            img_w, img_h, out_width, out_height,
        )

        # Blend in linear light at the frame's own exposure. Values above 1.0 are
        # kept: the tone map at the end is what decides where white sits.
        projected = _srgb_to_linear(projected) * total_scales[i]

        # Only a token feather here — just enough to antialias the seam. Choosing
        # the blend width per frequency is the pyramid's job, not this mask's.
        # Re-masked by `valid` afterwards: the blur spreads weight a few pixels
        # past the edge of what this frame actually saw, and weight out there
        # would extend the coverage mask over black pixels — which the pole fill
        # then happily propagates up into the sky as dark scallops.
        weight = cv2.GaussianBlur((region & valid).astype(np.float32), (0, 0), 2.0)
        weight *= valid
        del region

        blender.add(projected, valid, weight)
        weight_total += weight
        covered |= valid

        del projected, valid, weight
        print(f"\r  Blending... {(i + 1) * 100 // n}%", end="", flush=True)

    print()

    output = blender.result()
    del blender
    # Coverage is the union of what the frames saw, not where blend weight landed.
    mask = covered & (weight_total > 1e-3)
    output[~mask] = 0

    print("  Filling zenith...")
    photographed = mask
    output, mask = _fill_pole(output, mask)

    print("  Tone mapping...")
    output = _tonemap(output, mask, stats_mask=photographed)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), output, [cv2.IMWRITE_JPEG_QUALITY, 95])
    # Without this the file is just a wide JPEG as far as every viewer is
    # concerned; GPano is what marks it as a full equirectangular sphere.
    _inject_xmp_pano(output_path, out_width, out_height)
    print(f"  Saved to {output_path} ({out_width}x{out_height}, 360 metadata)")
    return str(output_path)


def stitch_opencv(image_paths, output_path):
    images = []
    for p in image_paths:
        img = cv2.imread(str(p))
        if img is None:
            print(f"Warning: Could not read {p}, skipping.")
            continue
        images.append(img)

    if len(images) < 2:
        print("Error: Could not load enough valid images.")
        return None

    print(f"Stitching {len(images)} images with OpenCV...")
    stitcher = cv2.Stitcher.create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(images)

    if status == cv2.Stitcher_OK:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output), pano)
        print(f"Panorama saved to {output}")
        return str(output)

    errors = {
        cv2.Stitcher_ERR_NEED_MORE_IMGS: "Not enough keypoint matches — images may not overlap enough.",
        cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL: "Homography estimation failed — images may not overlap.",
        cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "Camera parameter adjustment failed.",
    }
    print(f"Stitching failed: {errors.get(status, f'Unknown error ({status})')}")
    return None


def prepare_editor_data(image_paths, editor_dir="output/editor", out_width=8192, out_height=4096, reference_path=None):
    editor_dir = Path(editor_dir)
    editor_dir.mkdir(parents=True, exist_ok=True)

    entries = []
    for path in image_paths:
        xmp = read_dji_xmp(str(path))
        if xmp is None:
            continue
        img = cv2.imread(str(path))
        if img is None:
            continue
        entries.append({
            "path": str(path),
            "img": img,
            "yaw": xmp["GimbalYawDegree"],
            "pitch": xmp["GimbalPitchDegree"],
            "roll": xmp.get("GimbalRollDegree", 0.0),
        })

    if len(entries) < 2:
        print("Not enough valid DJI images.")
        return None

    h_fov, v_fov = get_camera_fov(str(image_paths[0]))
    if h_fov is None:
        print("Could not determine camera FOV from EXIF.")
        return None

    img_h, img_w = entries[0]["img"].shape[:2]
    print(f"  Camera FOV: {math.degrees(h_fov):.1f} x {math.degrees(v_fov):.1f} deg")
    print(f"  Input: {len(entries)} images at {img_w}x{img_h}")

    print("  Refining rotations (SIFT + bundle adjustment)...")
    # The focal length comes back fitted, not taken from EXIF — see refine_rotations.
    entries, fx, fy = refine_rotations(entries, h_fov, v_fov, img_w, img_h)

    cx, cy_img = img_w / 2.0, img_h / 2.0

    rotations = []
    for e in entries:
        R = rotation_matrix(
            math.radians(e["yaw"]),
            math.radians(e["pitch"]),
            math.radians(e["roll"]),
        )
        rotations.append(R.T)

    print("  Normalising exposure from EXIF...")
    exposure_scales = compute_exposure_scales([e["path"] for e in entries])

    print("  Projecting at half resolution...")
    small_layers, small_masks, sw, sh = _project_lowres(
        entries, rotations, fx, fy, cx, cy_img, img_w, img_h, out_width, out_height, scale=2)

    n = len(entries)
    print("  Solving residual exposure gains from overlaps...")
    gains = _compute_exposure_gains(small_layers, small_masks, n,
                                    prescale=exposure_scales)
    # Cached as one number per image: the editor's render path applies it in
    # linear light exactly as stitch_dji does.
    total_scales = exposure_scales * gains
    for i in range(n):
        small_layers[i] = _rescale_srgb(small_layers[i], total_scales[i])

    print("  Finding initial seams (Voronoi)...")
    seam_labels = _find_seams(small_masks, sw, sh, n)

    print("  Detecting people...")
    detections = _detect_people(entries, fx, fy, cx, cy_img, out_width, out_height)
    print("  Preserving subjects...")
    seam_labels = _preserve_subjects(seam_labels, small_layers, small_masks, detections, n,
                                     out_width, out_height)

    print("  Saving editor data...")
    for i in range(n):
        cv2.imwrite(str(editor_dir / f"layer_{i:02d}.png"), small_layers[i])
        cv2.imwrite(str(editor_dir / f"mask_{i:02d}.png"), small_masks[i])
        src = entries[i]["img"]
        thumb = cv2.resize(src, (src.shape[1] // 2, src.shape[0] // 2),
                           interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(editor_dir / f"source_{i:02d}.jpg"), thumb,
                    [cv2.IMWRITE_JPEG_QUALITY, 85])

    seam_img = np.where(seam_labels >= 0, seam_labels, 255).astype(np.uint8)
    cv2.imwrite(str(editor_dir / "seams.png"), seam_img)

    if reference_path:
        print(f"  Using reference image as starting composite: {reference_path}")
        ref = cv2.imread(str(reference_path))
        if ref is not None:
            ref_full = cv2.resize(ref, (sw, sh), interpolation=cv2.INTER_AREA)
            cv2.imwrite(str(editor_dir / "composite.jpg"), ref_full, [cv2.IMWRITE_JPEG_QUALITY, 92])
        else:
            print(f"  WARNING: Could not read reference image, falling back to blended composite")
            reference_path = None

    if not reference_path:
        print("  Generating blended composite...")
        blend_sigma = 10.0
        comp = np.zeros((sh, sw, 3), dtype=np.float32)
        comp_w = np.zeros((sh, sw), dtype=np.float32)
        for i in range(n):
            region = (seam_labels == i).astype(np.float32)
            w = cv2.GaussianBlur(region, (0, 0), blend_sigma)
            comp += small_layers[i].astype(np.float32) * w[:, :, np.newaxis]
            comp_w += w
        comp_w_safe = np.where(comp_w > 1e-10, comp_w, 1.0)
        for c_ch in range(3):
            comp[:, :, c_ch] = np.where(comp_w > 1e-10, comp[:, :, c_ch] / comp_w_safe, 0)
        comp = np.clip(comp, 0, 255).astype(np.uint8)
        cv2.imwrite(str(editor_dir / "composite.jpg"), comp, [cv2.IMWRITE_JPEG_QUALITY, 92])

    metadata = {
        "n_images": n,
        "width": int(sw),
        "height": int(sh),
        "full_width": out_width,
        "full_height": out_height,
        "h_fov": h_fov,
        "v_fov": v_fov,
        "img_w": img_w,
        "img_h": img_h,
        "images": [],
    }
    for i, e in enumerate(entries):
        metadata["images"].append({
            "index": i,
            "name": Path(e["path"]).name,
            "yaw": e["yaw"],
            "pitch": e["pitch"],
            "roll": e["roll"],
            "gain": float(total_scales[i]),
        })

    with open(editor_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    cache = {
        "image_paths": [e["path"] for e in entries],
        "rotations": rotations,
        "gains": total_scales.tolist() if isinstance(total_scales, np.ndarray) else list(total_scales),
        "fx": fx, "fy": fy, "cx": cx, "cy_img": cy_img,
        "img_w": img_w, "img_h": img_h,
    }
    with open(editor_dir / "cache.pkl", "wb") as f:
        pickle.dump(cache, f)

    print(f"  Editor data saved to {editor_dir}/ ({n} layers, {sw}x{sh})")
    return metadata


def reproject_all(editor_dir, rotation_updates, out_width=8192, out_height=4096):
    editor_dir = Path(editor_dir)
    with open(editor_dir / "cache.pkl", "rb") as f:
        cache = pickle.load(f)
    with open(editor_dir / "metadata.json") as f:
        meta = json.load(f)

    fx, fy = cache["fx"], cache["fy"]
    cx, cy_img = cache["cx"], cache["cy_img"]
    img_w, img_h = cache["img_w"], cache["img_h"]
    gains = cache["gains"]
    n = meta["n_images"]

    for upd in rotation_updates:
        i = upd["index"]
        meta["images"][i]["yaw"] = upd["yaw"]
        meta["images"][i]["pitch"] = upd["pitch"]
        if "roll" in upd:
            meta["images"][i]["roll"] = upd["roll"]

    entries = []
    rotations = []
    for i in range(n):
        img_data = meta["images"][i]
        img = cv2.imread(cache["image_paths"][i])
        entries.append({
            "img": img,
            "yaw": img_data["yaw"],
            "pitch": img_data["pitch"],
            "roll": img_data["roll"],
        })
        R = rotation_matrix(
            math.radians(img_data["yaw"]),
            math.radians(img_data["pitch"]),
            math.radians(img_data["roll"]),
        )
        rotations.append(R.T)

    print("  Re-projecting at half resolution...")
    small_layers, small_masks, sw, sh = _project_lowres(
        entries, rotations, fx, fy, cx, cy_img, img_w, img_h, out_width, out_height, scale=2)

    # Normalise after projection, in linear light with a highlight rolloff, so the
    # preview layers match what the full-res render will produce.
    for i in range(n):
        small_layers[i] = _rescale_srgb(small_layers[i], gains[i])

    print("  Finding seams...")
    seam_labels = _find_seams(small_masks, sw, sh, n)

    print("  Detecting people...")
    detections = _detect_people(entries, fx, fy, cx, cy_img, out_width, out_height)
    print("  Preserving subjects...")
    seam_labels = _preserve_subjects(seam_labels, small_layers, small_masks, detections, n,
                                     out_width, out_height)

    for i in range(n):
        cv2.imwrite(str(editor_dir / f"layer_{i:02d}.png"), small_layers[i])
        cv2.imwrite(str(editor_dir / f"mask_{i:02d}.png"), small_masks[i])

    seam_img = np.where(seam_labels >= 0, seam_labels, 255).astype(np.uint8)
    cv2.imwrite(str(editor_dir / "seams.png"), seam_img)

    print("  Generating blended composite...")
    blend_sigma = 10.0
    comp = np.zeros((sh, sw, 3), dtype=np.float32)
    comp_w = np.zeros((sh, sw), dtype=np.float32)
    for i in range(n):
        region = (seam_labels == i).astype(np.float32)
        w = cv2.GaussianBlur(region, (0, 0), blend_sigma)
        comp += small_layers[i].astype(np.float32) * w[:, :, np.newaxis]
        comp_w += w
    comp_w_safe = np.where(comp_w > 1e-10, comp_w, 1.0)
    for c_ch in range(3):
        comp[:, :, c_ch] = np.where(comp_w > 1e-10, comp[:, :, c_ch] / comp_w_safe, 0)
    comp = np.clip(comp, 0, 255).astype(np.uint8)
    cv2.imwrite(str(editor_dir / "composite.jpg"), comp, [cv2.IMWRITE_JPEG_QUALITY, 92])

    meta["width"] = int(sw)
    meta["height"] = int(sh)
    if "crop" in meta:
        del meta["crop"]
    with open(editor_dir / "metadata.json", "w") as f:
        json.dump(meta, f, indent=2)

    cache["rotations"] = rotations
    with open(editor_dir / "cache.pkl", "wb") as f:
        pickle.dump(cache, f)

    print(f"  Re-projection complete ({n} layers, {sw}x{sh})")
    return meta


def composite_from_seams(editor_dir, seam_labels_img):
    editor_dir = Path(editor_dir)
    with open(editor_dir / "metadata.json") as f:
        meta = json.load(f)

    n = meta["n_images"]
    sw, sh = meta["width"], meta["height"]

    seam_labels = cv2.resize(seam_labels_img, (sw, sh),
                             interpolation=cv2.INTER_NEAREST).astype(np.int16)
    seam_labels[seam_labels == 255] = -1

    layers = []
    masks = []
    for i in range(n):
        layer = cv2.imread(str(editor_dir / f"layer_{i:02d}.png"))
        mask = cv2.imread(str(editor_dir / f"mask_{i:02d}.png"), cv2.IMREAD_GRAYSCALE)
        layers.append(layer)
        masks.append(mask)

    blend_sigma = 10.0
    margin = int(3.5 * blend_sigma)

    output = np.zeros((sh, sw, 3), dtype=np.float32)
    weight_total = np.zeros((sh, sw), dtype=np.float32)

    active_set = set(int(v) for v in np.unique(seam_labels) if v >= 0)

    for i in active_set:
        region = (seam_labels == i).astype(np.float32)
        w = cv2.GaussianBlur(region, (0, 0), blend_sigma)
        output += layers[i].astype(np.float32) * w[:, :, np.newaxis]
        weight_total += w

    safe_wt = np.where(weight_total > 1e-10, weight_total, 1.0)
    for c in range(3):
        output[:, :, c] = np.where(weight_total > 1e-10, output[:, :, c] / safe_wt, 0)

    output = np.clip(output, 0, 255).astype(np.uint8)
    _, jpeg_data = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return jpeg_data.tobytes()


def render_with_seams(editor_dir, seam_labels_img, output_path,
                      out_width=8192, out_height=4096, cancel_event=None):
    editor_dir = Path(editor_dir)

    with open(editor_dir / "cache.pkl", "rb") as f:
        cache = pickle.load(f)
    with open(editor_dir / "metadata.json") as f:
        meta = json.load(f)

    image_paths = cache["image_paths"]
    rotations = cache["rotations"]
    gains = cache["gains"]
    fx, fy = cache["fx"], cache["fy"]
    cx, cy_img = cache["cx"], cache["cy_img"]
    img_w, img_h = cache["img_w"], cache["img_h"]

    crop = meta.get("crop")
    if crop:
        render_x0, render_y0 = crop["x0"], crop["y0"]
        render_x1, render_y1 = crop["x1"], crop["y1"]
        render_w = render_x1 - render_x0
        render_h = render_y1 - render_y0
        uncropped_w = crop["uncropped_w"]
        uncropped_h = crop["uncropped_h"]
    else:
        render_x0, render_y0 = 0, 0
        render_w, render_h = out_width, out_height
        render_x1, render_y1 = out_width, out_height
        uncropped_w, uncropped_h = out_width, out_height

    entries = []
    for i, path in enumerate(image_paths):
        img = cv2.imread(path)
        if img is None:
            print(f"  Warning: Could not read {path}")
            return None
        entries.append({"img": img})

    n = len(entries)

    seam_labels_full = cv2.resize(seam_labels_img, (render_w, render_h),
                                  interpolation=cv2.INTER_NEAREST).astype(np.int16)
    seam_labels_full[seam_labels_full == 255] = -1

    # The pole fill reasons about latitude and wraps rows at the 360 deg seam, so
    # it — and the blender's circular padding — only apply to an uncropped sphere.
    full_sphere = (render_x0 == 0 and render_y0 == 0
                   and render_w == uncropped_w and render_h == uncropped_h)

    # Same blending as stitch_dji: multi-band in linear light at each frame's own
    # exposure, tone mapped once at the end. The painted seam labels still decide
    # which frame owns each pixel — the pyramid only governs how wide the handover
    # is at each spatial frequency, so a seam drawn around an object is respected.
    gpu_images = [cv2.cuda_GpuMat(e["img"]) for e in entries] if USE_CUDA else None
    print(f"  Rendering at {render_w}x{render_h} "
          f"({'CUDA, ' if USE_CUDA else ''}{MULTIBAND_LEVELS}-band)...")

    blender = _BandBlender(render_h, render_w, wrap=full_sphere)
    weight_total = np.zeros((render_h, render_w), dtype=np.float32)
    covered = np.zeros((render_h, render_w), dtype=bool)

    for i in range(n):
        if cancel_event and cancel_event.is_set():
            print("\n  Render cancelled.")
            return None

        region = seam_labels_full == i
        if not np.any(region):
            continue

        projected, valid = _project_full(
            entries[i]["img"], rotations[i], fx, fy, cx, cy_img, img_w, img_h,
            uncropped_w, uncropped_h, x0=render_x0, x1=render_x1,
            y0=render_y0, y1=render_y1,
            gpu_img=gpu_images[i] if gpu_images else None,
        )
        projected = _srgb_to_linear(projected) * gains[i]

        # Masked by `valid` after blurring: weight past the edge of what a frame
        # saw would extend the coverage mask over black pixels.
        weight = cv2.GaussianBlur((region & valid).astype(np.float32), (0, 0), 2.0)
        weight *= valid
        del region

        blender.add(projected, valid, weight)
        weight_total += weight
        covered |= valid

        del projected, valid, weight
        print(f"\r  Blending... {(i + 1) * 100 // n}%", end="", flush=True)

    output = blender.result()
    del blender
    mask = covered & (weight_total > 1e-3)
    output[~mask] = 0
    covered = mask

    print()
    photographed = covered
    if full_sphere:
        print("  Filling zenith...")
        output, covered = _fill_pole(output, covered)

    print("  Tone mapping...")
    output = _tonemap(output, covered, stats_mask=photographed)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), output, [cv2.IMWRITE_JPEG_QUALITY, 95])
    _inject_xmp_pano(output_path, render_w, render_h)
    _generate_viewer_html(output_path)
    print(f"  Saved to {output_path} ({render_w}x{render_h}, 360 metadata)")
    return str(output_path)


def _generate_viewer_html(jpeg_path):
    """Generate a standalone HTML file with embedded panorama for 360 viewing."""
    import base64
    jpeg_path = Path(jpeg_path)
    html_path = jpeg_path.with_suffix('.html')
    image_name = jpeg_path.name
    b64 = base64.b64encode(jpeg_path.read_bytes()).decode('ascii')
    html = f'''<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>360 Panorama - {image_name}</title>
<style>body{{margin:0;overflow:hidden}}canvas{{display:block}}</style>
</head><body>
<script type="importmap">{{"imports":{{"three":"https://cdn.jsdelivr.net/npm/three@0.160/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160/examples/jsm/"}}}}</script>
<script type="module">
import * as THREE from 'three';
import {{OrbitControls}} from 'three/addons/controls/OrbitControls.js';
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, innerWidth/innerHeight, 0.1, 100);
camera.position.set(0, 0, 0.01);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableZoom = false;
controls.enablePan = false;
controls.rotateSpeed = -0.3;
controls.enableDamping = true;
controls.dampingFactor = 0.08;
let targetFov = camera.fov;
renderer.domElement.addEventListener('wheel', (e) => {{
  e.preventDefault();
  targetFov = Math.max(20, Math.min(120, targetFov + e.deltaY * 0.05));
}}, {{passive: false}});
const geo = new THREE.SphereGeometry(50, 64, 32);
geo.scale(-1, 1, 1);
const img = new Image();
img.onload = () => {{
  const tex = new THREE.Texture(img);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  const mat = new THREE.MeshBasicMaterial({{map: tex}});
  scene.add(new THREE.Mesh(geo, mat));
}};
img.src = 'data:image/jpeg;base64,{b64}';
addEventListener('resize', () => {{
  camera.aspect = innerWidth/innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
}});
function animate() {{
  requestAnimationFrame(animate);
  controls.update();
  if (Math.abs(camera.fov - targetFov) > 0.01) {{
    camera.fov += (targetFov - camera.fov) * 0.1;
    camera.updateProjectionMatrix();
  }}
  renderer.render(scene, camera);
}}
animate();
</script>
</body></html>'''
    html_path.write_text(html)
    print(f"  Viewer: {html_path}")


def _inject_xmp_pano(jpeg_path, width, height):
    """Inject GPano XMP metadata so viewers recognize this as a 360 panorama."""
    xmp = (
        '<?xpacket begin="\xef\xbb\xbf" id="W5M0MpCehiHzreSzNTczkc9d"?>'
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        '<rdf:Description rdf:about="" '
        'xmlns:GPano="http://ns.google.com/photos/1.0/panorama/" '
        f'GPano:ProjectionType="equirectangular" '
        f'GPano:UsePanoramaViewer="True" '
        f'GPano:CroppedAreaImageWidthPixels="{width}" '
        f'GPano:CroppedAreaImageHeightPixels="{height}" '
        f'GPano:FullPanoWidthPixels="{width}" '
        f'GPano:FullPanoHeightPixels="{height}" '
        f'GPano:CroppedAreaLeftPixels="0" '
        f'GPano:CroppedAreaTopPixels="0" '
        '/>'
        '</rdf:RDF>'
        '</x:xmpmeta>'
        '<?xpacket end="w"?>'
    ).encode('utf-8')

    with open(jpeg_path, 'rb') as f:
        data = f.read()

    if data[:2] != b'\xff\xd8':
        return

    # Insert XMP APP1 marker after SOI
    xmp_header = b'http://ns.adobe.com/xap/1.0/\x00'
    app1_payload = xmp_header + xmp
    app1_length = len(app1_payload) + 2  # +2 for the length field itself
    app1_marker = b'\xff\xe1' + app1_length.to_bytes(2, 'big') + app1_payload

    # Remove existing XMP APP1 if present
    pos = 2
    clean = data[:2]
    while pos < len(data) - 1:
        if data[pos] != 0xFF:
            clean += data[pos:]
            break
        marker = data[pos:pos + 2]
        if marker == b'\xff\xda':
            clean += data[pos:]
            break
        seg_len = int.from_bytes(data[pos + 2:pos + 4], 'big')
        segment = data[pos:pos + 2 + seg_len]
        if marker == b'\xff\xe1' and b'http://ns.adobe.com/xap/1.0/' in segment:
            pass  # skip existing XMP
        else:
            clean += segment
        pos += 2 + seg_len

    # Insert new XMP after SOI
    out = clean[:2] + app1_marker + clean[2:]
    with open(jpeg_path, 'wb') as f:
        f.write(out)


def stitch_images(image_paths, output_path="output/panorama.jpg"):
    if len(image_paths) < 2:
        print("Error: Need at least 2 images to stitch.")
        return None

    if has_dji_metadata(image_paths):
        return stitch_dji(image_paths, output_path)
    return stitch_opencv(image_paths, output_path)
