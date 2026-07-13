"""ROS 2 mesajlarini numpy'a ceviren yardimcilar (cv_bridge'siz — hosttaki cv2
numpy 2.x ile uyumsuz oldugu icin PIL + numpy kullanilir)."""

import numpy as np


def image_to_np(msg):
    """sensor_msgs/Image -> numpy dizisi (H,W[,C]). bgra8, bgr8, 32FC1 destekli."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc == 'bgra8':
        img = buf.reshape(h, msg.step // 4, 4)[:, :w, :]
        return img[:, :, [2, 1, 0]]  # BGRA -> RGB
    if enc == 'bgr8':
        img = buf.reshape(h, msg.step // 3, 3)[:, :w, :]
        return img[:, :, ::-1]
    if enc == 'rgb8':
        return buf.reshape(h, msg.step // 3, 3)[:, :w, :]
    if enc == '32fc1':
        f = np.frombuffer(bytes(msg.data), dtype=np.float32)
        return f.reshape(h, msg.step // 4)[:, :w]
    if enc in ('mono8', '8uc1'):
        return buf.reshape(h, msg.step)[:, :w]
    if enc in ('mono16', '16uc1'):
        u = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        return u.reshape(h, msg.step // 2)[:, :w]
    raise ValueError(f'desteklenmeyen encoding: {msg.encoding}')


def pointcloud2_to_xyzrgb(msg):
    """PointCloud2 (x,y,z,rgb float32) -> (xyz float32 [N,3], rgb uint8 [N,3]).

    row_step dolgulu bulutlari da dogru cozumler; NaN satirlari atilir.
    """
    offsets = {f.name: f.offset for f in msg.fields}
    prefix = '>' if msg.is_bigendian else '<'
    names, formats, offs = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(prefix + 'f4')
        offs.append(f.offset)
    dtype = np.dtype({'names': names, 'formats': formats,
                      'offsets': offs, 'itemsize': msg.point_step})
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    height = msg.height or 1
    row_bytes = msg.width * msg.point_step
    if msg.row_step and msg.row_step != row_bytes and height > 1:
        data = data.reshape(height, msg.row_step)[:, :row_bytes].reshape(-1)
    count = msg.width * height
    cloud = np.frombuffer(data.tobytes(), dtype=dtype, count=count)

    xyz = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1).astype(np.float32)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if 'rgb' in offsets:
        packed = np.ascontiguousarray(cloud['rgb'][finite]).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF,
                        packed & 0xFF], axis=1).astype(np.uint8)
    else:
        rgb = np.full((xyz.shape[0], 3), 255, dtype=np.uint8)
    return xyz, rgb


def quat_to_rpy(qx, qy, qz, qw):
    """Kuaterniyon -> roll/pitch/yaw (radyan)."""
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr, cosr)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny, cosy)
    return float(roll), float(pitch), float(yaw)


def make_lut(hex_steps):
    """Hex renk listesinden 256'lik dogrusal interpolasyonlu LUT uret [256,3] u8."""
    cols = np.array([[int(h[1:3], 16), int(h[3:5], 16), int(h[5:7], 16)]
                     for h in hex_steps], dtype=np.float32)
    xs = np.linspace(0, len(cols) - 1, 256)
    lut = np.empty((256, 3), dtype=np.uint8)
    for c in range(3):
        lut[:, c] = np.interp(xs, np.arange(len(cols)), cols[:, c]).astype(np.uint8)
    return lut


# dataviz paletinin mavi sequential rampasi (100 -> 700, acik=yakin/az koyu=uzak/cok)
BLUE_RAMP = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
# ikinci sequential baglam icin aqua tonlari (confidence gibi)
AQUA_RAMP = ['#d3f2e5', '#8fdec0', '#4cc79a', '#1baf7a', '#128a60', '#0a6647', '#054630']
