#!/bin/bash
# ZED kamerasına ait asılı kalmış paylaşımlı bellek bloklarını temizler

echo "[CLEANUP] Paylaşımlı bellek (Shared Memory) temizleniyor..."
rm -f /dev/shm/zed_rgb
rm -f /dev/shm/zed_depth
rm -f /dev/shm/zed_meta
echo "[CLEANUP] Temizlik tamamlandı."