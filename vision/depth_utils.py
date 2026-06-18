import numpy as np


def get_distance_from_bbox(depth_array, bbox, method="median"):
    """
    Derinlik matrisi ve sınırlayıcı kutu (bounding box) koordinatlarını kullanarak
    nesnenin kameraya olan mesafesini hesaplar.

    Argümanlar:
        depth_array (np.ndarray): ZED'den gelen 2 boyutlu derinlik matrisi (H, W).
        bbox (list or tuple): [x1, y1, x2, y2] formatında bounding box koordinatları.
        method (str): "median" (varsayılan) veya "mean" (ortalama) hesaplama yöntemi.

    Dönüş:
        float: Hesaplanmış mesafe (metre cinsinden). Geçersiz/hatalı durumlarda -1.0 döner.
    """
    if depth_array is None or bbox is None:
        return -1.0

    x1, y1, x2, y2 = map(int, bbox)
    h, w = depth_array.shape

    # Bounding box koordinatlarını derinlik matrisi sınırlarına (0 ile w/h arasına) kırp
    x1_c, x2_c = max(0, x1), min(w, x2)
    y1_c, y2_c = max(0, y1), min(h, y2)

    # Kırpma sonrası geçerli bir alan (Area > 0) kalıp kalmadığını kontrol et
    if y2_c <= y1_c or x2_c <= x1_c:
        return -1.0

    # Kutu içerisindeki derinlik piksellerini (ROI) al
    roi_depth = depth_array[y1_c:y2_c, x1_c:x2_c]

    # Seçilen yönteme göre mesafeyi hesapla
    # np.nanmedian ve np.nanmean, matris içindeki NaN değerleri (boş pikselleri) yoksayar
    if method == "median":
        distance = float(np.nanmedian(roi_depth))
    elif method == "mean":
        distance = float(np.nanmean(roi_depth))
    else:
        distance = float(np.nanmedian(roi_depth))  # Fallback

    # Hesaplanan değer Sonsuz (Inf) veya Tanımsız (NaN) ise -1.0 döndür
    if not np.isfinite(distance):
        return -1.0

    return distance


def is_valid_distance(distance, min_dist=0.3, max_dist=20.0):
    """
    Ölçülen mesafenin ZED kamerasının çalışma aralığında olup olmadığını doğrular.
    Gerektiğinde filtreleme yapmak için kullanılabilir.
    """
    if distance <= 0.0:
        return False
    return min_dist <= distance <= max_dist