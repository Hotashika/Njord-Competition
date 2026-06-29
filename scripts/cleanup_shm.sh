#!/bin/bash

echo "[CLEANUP] Shared Memory temizleniyor..."
rm -f /dev/shm/RGB_DATA
rm -f /dev/shm/DEPTH_DATA
rm -f /dev/shm/ZED_META
echo "[CLEANUP] Temizlik tamamlandı."
