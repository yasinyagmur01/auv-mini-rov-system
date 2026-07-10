# Şerit takibi durum bilgisi
std_msgs/Header header

uint8 SEARCHING=0        # şerit henüz bulunamadı
uint8 TRACKING=1         # şerit takip ediliyor
uint8 LOST=2             # şerit kaybedildi
uint8 END_OF_LINE=3      # şeridin sonu tespit edildi

uint8 status
float32 lateral_offset   # -1..1, görüntü merkezine göre normalize yanal sapma
float32 angle_deg        # şeridin görüntüdeki açısı (0 = tam ileri)
float32 confidence       # 0..1 tespit güveni
