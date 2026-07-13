# MAVLink MANUAL_CONTROL karşılığı (ArduSub eksenleri)
std_msgs/Header header

int16 x    # ileri/geri  -1000..1000 (0 nötr)
int16 y    # yanal (sancak +) -1000..1000
int16 z    # dikey 0..1000 (500 nötr, >500 yukarı)
int16 r    # yaw -1000..1000 (saat yönü +)
