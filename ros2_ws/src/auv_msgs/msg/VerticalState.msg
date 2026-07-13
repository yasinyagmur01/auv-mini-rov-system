# Net dikey eksen ("Y ekseni") verisi: Bar30 + Ping Sonar birleşimi
std_msgs/Header header

float32 depth_m          # Bar30: su yüzeyinden derinlik (metre, aşağı pozitif)
float32 altitude_m       # Ping Sonar: deniz tabanından yükseklik (metre)
float32 water_column_m   # depth_m + altitude_m = aracın altındaki taban derinliği

bool depth_valid         # Bar30 verisi taze ve mantıklı mı
bool altitude_valid      # Ping verisi menzil sınırları içinde ve taze mi
