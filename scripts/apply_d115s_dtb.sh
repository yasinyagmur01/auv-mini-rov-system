#!/usr/bin/env bash
# ============================================================================
# AVerMedia D115S device-tree uygulamasi (Yol 1 — geri alinabilir DTB degisimi)
#
# SUDO GEREKTIRIR:  sudo bash scripts/apply_d115s_dtb.sh
#
# Ne yapar:
#  1. AVerMedia BSP'den (JetPack 6.2.2 / R1.4.0) cikarilan D115S DTB'sini
#     /boot/dtb/ altina kopyalar (Orin NX 16GB: p3767-0000 varyanti).
#  2. extlinux.conf'u yedekler (extlinux.conf.devkit.bak).
#  3. LABEL primary'ye FDT satiri ekler.
#  4. Eski davranisi koruyan 'devkit-fallback' LABEL'i ekler (FDT'siz) —
#     boot menusunden secilebilir guvenlik agi.
#
# GERI ALMA: sudo cp /boot/extlinux/extlinux.conf.devkit.bak /boot/extlinux/extlinux.conf
#
# Gerekce: cihazdaki devkit DTB'sinde usb3-2 lane'i 'disabled' ve usb2/usb3
# eslesmeleri farkli; D115S DTB'sinde usb3-2 'okay' (dtc ile dogrulandi).
# SuperSpeed bu yuzden hic kurulamiyordu.
# ============================================================================
set -euo pipefail

# Test icin gecersiz kilinabilir (or: BOOT_DIR=/tmp/test bash -x ...)
BOOT_DIR="${BOOT_DIR:-/boot}"
SRC_DTB="${SRC_DTB:-/home/qayra/bsp_tmp/JetPack_6.2.2_Linux_JETSON_desktop/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0000-nv-d115s.dtb}"

DTB_NAME="tegra234-p3768-0000+p3767-0000-nv-d115s.dtb"
CONF="$BOOT_DIR/extlinux/extlinux.conf"
BAK="$BOOT_DIR/extlinux/extlinux.conf.devkit.bak"

if [ "${BOOT_DIR}" = "/boot" ] && [ "$(id -u)" -ne 0 ]; then
    echo "Bu script sudo ile calistirilmalidir: sudo bash $0" >&2
    exit 1
fi

[ -f "$SRC_DTB" ] || { echo "HATA: Kaynak DTB yok: $SRC_DTB" >&2; exit 1; }
[ -f "$CONF" ]    || { echo "HATA: $CONF yok" >&2; exit 1; }

# DTB butunluk kontrolu (dtc ile acilabiliyor mu?)
if command -v dtc >/dev/null 2>&1; then
    dtc -I dtb -O dts -o /dev/null "$SRC_DTB" 2>/dev/null || {
        echo "HATA: DTB dogrulanamadi (bozuk dosya?)" >&2; exit 1; }
fi

if grep -q "^      FDT " "$CONF"; then
    echo "UYARI: extlinux.conf'ta zaten FDT satiri var; dokunulmadi." >&2
    grep "^      FDT" "$CONF"
    exit 1
fi

echo "==> [1/4] DTB kopyalaniyor: $BOOT_DIR/dtb/$DTB_NAME"
cp "$SRC_DTB" "$BOOT_DIR/dtb/$DTB_NAME"

echo "==> [2/4] extlinux.conf yedekleniyor: $BAK"
cp "$CONF" "$BAK"

echo "==> [3/4] LABEL primary'ye FDT satiri ekleniyor..."
sed -i "s|^      INITRD /boot/initrd$|      INITRD /boot/initrd\n      FDT /boot/dtb/$DTB_NAME|" "$CONF"

grep -q "^      FDT /boot/dtb/$DTB_NAME" "$CONF" || {
    echo "HATA: FDT satiri eklenemedi; yedek geri yukleniyor." >&2
    cp "$BAK" "$CONF"; exit 1; }

echo "==> [4/4] 'devkit-fallback' boot girdisi ekleniyor (FDT'siz guvenlik agi)..."
APPEND_LINE=$(grep -m1 "^      APPEND" "$CONF")
cat >> "$CONF" <<EOF

LABEL devkit-fallback
      MENU LABEL devkit DTB fallback (FDT'siz eski durum)
      LINUX /boot/Image
      INITRD /boot/initrd
$APPEND_LINE
EOF

echo ""
echo "================== SONUC =================="
cat "$CONF"
echo "==========================================="
echo "Tamam. Simdi yeniden baslatin: sudo reboot"
echo "Dogrulama (reboot sonrasi): bash scripts/check_zed_usb.sh"
echo "GERI ALMA: sudo cp $BAK $CONF && sudo reboot"
