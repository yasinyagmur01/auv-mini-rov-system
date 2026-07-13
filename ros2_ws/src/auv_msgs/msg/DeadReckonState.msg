# Ölü hesap (dead reckoning) konum kestirimi
std_msgs/Header header

bool origin_valid        # yüzeyde GPS fix alındı mı
float64 origin_lat       # başlangıç fix'i (derece)
float64 origin_lon

float32 x_east_m         # başlangıca göre doğu (metre)
float32 y_north_m        # başlangıca göre kuzey (metre)
float32 heading_deg      # kullanılan pruva (derece, 0=Kuzey)
float32 speed_mps        # hız tablosundan kestirilen anlık hız
float32 distance_total_m # toplam kat edilen yol

float64 est_lat          # kestirilen mutlak konum
float64 est_lon
