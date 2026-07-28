@echo off
chcp 65001 >nul
REM Tek komut: Jetson'a SSH yapar ve saha menüsünü açar. Cikis: menude q.
ssh -t qayra@192.168.2.135 "bash ~/auv/scripts/saha_menu.sh"
