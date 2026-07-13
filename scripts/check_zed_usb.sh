#!/usr/bin/env bash
# ZED 2i USB baglanti kontrolu.
# Kameranin video arayuzu (2b03:f880) SADECE USB 3.x baglantisinda numaralanir.
# Sensor arayuzu (2b03:f881 HID) USB 2.0'da da gorunur — tek basina yeterli DEGILDIR.
set -u

echo "== ZED 2i USB tani =="

HID=$(lsusb -d 2b03:f881 2>/dev/null || true)
CAM=$(lsusb -d 2b03:f880 2>/dev/null || true)

if [ -n "$CAM" ]; then
    echo "[OK] Kamera arayuzu bulundu: $CAM"
else
    echo "[HATA] Kamera arayuzu (2b03:f880) YOK."
fi

if [ -n "$HID" ]; then
    echo "[OK] Sensor (HID) arayuzu bulundu: $HID"
else
    echo "[HATA] Sensor arayuzu (2b03:f881) da yok — kamera hic bagli/beslenmiyor olabilir."
fi

echo ""
echo "-- USB topolojisi --"
lsusb -t

echo ""
if [ -n "$CAM" ]; then
    # Kameranin bagli oldugu bus'in gercek link hizini raporla
    BUS=$(echo "$CAM" | awk '{print $2}' | sed 's/^0*//')
    SPEED=$(lsusb -t 2>/dev/null | awk -v bus="0$BUS" '$0 ~ "Bus "bus {print $NF; exit}')
    echo "[BILGI] Kamera Bus $BUS uzerinde, kok hiz: ${SPEED:-bilinmiyor}"
    case "${SPEED:-}" in
        5000M|10000M|20000M)
            echo "[OK] USB 3.x hizi dogru. Video cihazlari: $(ls /dev/video* 2>/dev/null || echo 'henuz yok')"
            echo "Sonraki test: /usr/local/zed/tools/ZED_Diagnostic --all"
            exit 0 ;;
        *)
            echo "[UYARI] Kamera numaralanmis ama link hizi USB3 degil (${SPEED:-?})."
            echo "        Kabloyu/portu kontrol edin; performans dusuk olacaktir."
            exit 1 ;;
    esac
else
    cat <<'EOF'
[COZUM]
 1. ZED 2i'yi hub KULLANMADAN dogrudan Jetson tasiyici karttaki USB 3.x
    porta takin (mavi renkli ya da SS isaretli port).
 2. Mutlaka ORIJINAL Stereolabs USB 3.0 kablosunu kullanin ve konektorun
    TAM oturdugundan emin olun (kamera tarafindaki vidalari da kontrol edin).
 3. Kablonun her iki ucunu cikartip yeniden takin; 5 sn bekleyin.
 4. Bu scripti tekrar calistirin: bash scripts/check_zed_usb.sh
    Beklenen: 10000M/5000M hizli bus altinda 2b03:f880.
EOF
    exit 1
fi
