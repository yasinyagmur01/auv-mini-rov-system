#!/usr/bin/env bash
# =====================================================================
# AUV SAHA MENÜSÜ — Jetson'da çalışır. Kontrol PC'sinden tek komutla
# (SSH ile) açılır. Seçim yap → rapor → Enter → menüye dön.
# Salt-okunur: motor/arm/MAVLink yazması YOK.
# =====================================================================
HC="bash $HOME/auv/scripts/health_check.sh"

while true; do
  clear 2>/dev/null
  echo "==================================================="
  echo "            AUV  SAHA  KONTROL"
  echo "==================================================="
  echo "   1) Haberleşme kontrolü   (kartlar + FC + köprü)"
  echo "   2) Sensör verileri       (var/yok + neden)"
  echo "   3) Sensör — takılı olanları seç (-i)"
  echo "   4) Tam test              (haberleşme + sensör + log)"
  echo "   5) Jetson kabuğu         (Linux komutları)"
  echo "   q) Çıkış"
  echo "==================================================="
  read -rp "Seçim: " sec || exit 0
  echo
  case "$sec" in
    1) $HC --comms-only ;;
    2) $HC --sample 8 ;;
    3) $HC -i ;;
    4) $HC ;;
    5) echo ">> Jetson kabuğu — menüye dönmek için 'exit' yaz."; bash ;;
    q|Q|"") echo "Çıkılıyor..."; exit 0 ;;
    *) echo "Geçersiz seçim: $sec" ;;
  esac
  echo
  read -rp "↵ Enter ile menüye dön..." _ || exit 0
done
