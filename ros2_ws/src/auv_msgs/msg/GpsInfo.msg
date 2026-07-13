# GPS_RAW_INT özet bilgisi
std_msgs/Header header

uint8 fix_type           # 0-1: yok, 2: 2D, 3: 3D, 4+: DGPS/RTK
uint8 satellites
float32 hdop
float64 lat              # derece
float64 lon              # derece
