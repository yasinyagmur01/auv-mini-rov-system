#!/usr/bin/env python3
"""Hiz tablosu kalibrasyon yardimcisi (havuzda calistirilir).

Prosedur:
  1. Havuzda 10 m'lik olculu duz hat isaretleyin (kulvar cizgisi ideal).
  2. Arac ALT_HOLD'da sabit derinlikte, hat basinda pruvasi hatta bakarken:
       python3 calibrate_speed.py --throttle 300 --seconds 12
  3. Iki kronometreci hattin gecis suresini olcer -> hiz = mesafe / sure.
  4. Her gaz kademesi (200..700) icin cift yonde tekrarlanir, ortalama alinir.
  5. Sonuclar config/speed_table.csv'ye yazilir.

Bu betik MAVLink'e dogrudan baglanir (mavlink-router panel ucu 14552'yi
dinleyen ayri bir GCS gibi davranir); ROS calisiyor olmak zorunda degil.
DIKKAT: Betik calisirken mission node IDLE olmali (tek komut kaynagi kurali).
"""
import argparse
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='udpout:127.0.0.1:14551',
                    help='Jetson uzerinde calistirin (varsayilan router ucu)')
    ap.add_argument('--throttle', type=int, required=True, help='0..1000')
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--sysid', type=int, default=1)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.url, source_system=args.sysid,
                                   source_component=194)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    print(f'Gaz {args.throttle} -> {args.seconds} sn. 3 sn icinde basliyor...')
    time.sleep(3)

    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            m.mav.manual_control_send(args.sysid, args.throttle, 0, 500, 0, 0)
            time.sleep(0.1)
    finally:
        for _ in range(10):  # notr birak
            m.mav.manual_control_send(args.sysid, 0, 0, 500, 0, 0)
            time.sleep(0.1)
    print('Bitti. Gecis suresini kaydedin: hiz = mesafe(m) / sure(s)')


if __name__ == '__main__':
    main()
