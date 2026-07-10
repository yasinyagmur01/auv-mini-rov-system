"""Panel yapilandirmasi - saha kurulumuna gore duzenleyin."""

JETSON_IP = '192.168.2.3'
WS_URL = f'ws://{JETSON_IP}:8765'

# mavlink-router'in panele ittigi ucu dinle (QGC 14550 ile cakismaz)
MAVLINK_LISTEN = 'udpin:0.0.0.0:14552'

AUV_SYSID = 1
MINIROV_SYSID = 2

# Video kaynaklari: sirayla denenir (GStreamer varsa ilki, yoksa FFMPEG+SDP)
import os
_SDP_DIR = os.path.join(os.path.dirname(__file__), 'sdp')

D435_SOURCES = [
    ('gst', 'udpsrc port=5601 caps="application/x-rtp,media=video,'
            'encoding-name=H264,payload=96" ! rtph264depay ! h264parse ! '
            'avdec_h264 ! videoconvert ! appsink drop=true sync=false'),
    ('ffmpeg', os.path.join(_SDP_DIR, 'd435.sdp')),
]
MINIROV_SOURCES = [
    ('gst', 'udpsrc port=5602 caps="application/x-rtp,media=video,'
            'encoding-name=H264,payload=96" ! rtph264depay ! h264parse ! '
            'avdec_h264 ! videoconvert ! appsink drop=true sync=false'),
    ('ffmpeg', os.path.join(_SDP_DIR, 'minirov.sdp')),
]
