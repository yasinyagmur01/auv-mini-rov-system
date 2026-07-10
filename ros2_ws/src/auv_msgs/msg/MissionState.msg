# Görev durum makinesi çıktısı
std_msgs/Header header

uint8 STATE_IDLE=0
uint8 STATE_MANUAL=1
uint8 STATE_VIDEO_PATTERN=2
uint8 STATE_LANE_FOLLOW=3
uint8 STATE_AUTONOMOUS_NAV=4
uint8 STATE_ABORTED=5

uint8 state
string state_name        # okunabilir durum adı
string step_name         # aktif adımın adı (örn. "drive 16s")
uint16 step_index        # 0 tabanlı aktif adım no
uint16 step_count        # görevdeki toplam adım
float32 step_elapsed_s   # aktif adımda geçen süre
float32 mission_elapsed_s
string message           # son bilgi/uyarı metni
