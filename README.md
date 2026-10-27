# IHAILOS — TEKNOFEST 2026 Sabit Kanat Otonomi ve Balistik Görev Motoru

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

IHAILOS, **TEKNOFEST 2026 TÜBİTAK Sabit Kanat İHA Yarışması (Görev 2)** için geliştirilmiş; gerçek zamanlı görüntü işleme, WGS84 coğrafi koordinat kestirimi, 2D Kalman hedef takibi, rüzgar kompanzasyonlu balistik yük bırakma ve emniyet kilitli sonlu durum makinesini (FSM) içeren çok iş parçacıklı bir otonom görev yazılımıdır.

---

## 📌 Mimari & Modül Sorumlulukları

Sistem 5 ana iş parçacığı üzerinden kilitli bir veri yolu (`SharedBus`) ile haberleşir:

```
Camera + Pixhawk ────► vision_engine ────► estimator ────► mission_coordinator ────► MAVLink + comms_gateway
                             ▲                                     │
                             └──────────── TelemetryPacket ────────┘
```

1. **`vision_engine/`**: OpenCV kare yakalama, UTC mikrosaniye zaman damgası, Ultralytics YOLO çıkarımı ve **8-kare ardışık doğrulama (debounce)**.
2. **`estimator/`**: Pinhole ters izdüşümü ($K^{-1}$), $30^\circ$ pitch-down kamera montaj açısı düzeltmesi, LiDAR AGL zemin kesişimi, **WGS84 koordinat kestirimi**, rüzgar vektörü hesabı ($V_w = V_g - V_a$) ve **2D Kalman filtreli hedef takibi**.
3. **`mission_coordinator/`**:
   - Durum Makinesi: `SEARCH → TRACK_LOCK → APPROACH_PLAN → FINAL_RUN → RELEASE → VERIFY → RTL`.
   - Emniyet Kapıları (`SafetyGate`):
     - Uçuş modu $\in \{\text{AUTO}, \text{GUIDED}\}$.
     - $|\text{Roll}| < 5^\circ$ ve $|\text{Pitch}| < 5^\circ$.
     - Taze aşağı yönlü AGL LiDAR verisi $\le 120\text{ m}$ (SHGM tavan limiti).
     - Geofence koridoru içinde bulunma (Shapely poligon kontrolü).
   - Yük Bırakma: MAVLink `MAV_CMD_DO_SET_SERVO` (AUX 9, PWM 2000).
4. **`comms_gateway/`**: Hakem sunucusuna gerçek zamanlı JSON telemetri iletimi, kilit/bırakma anı fotoğrafları ve çevrimdışı SQLite/JSON giden kutusu (`outbox`).

---

## 🚀 Çalıştırma

### Bağımlılıklar
```bash
pip install -r requirements.txt
```

### 1. Sentetik / Test Koşusu (Donanımsız)
```bash
python main.py --synthetic --duration 10
```

### 2. Testleri Çalıştırma
```bash
python -m pytest tests/
```

### 3. Simülasyon veya Gerçek Uçak ile Koşu
```bash
python main.py --mavlink udp:127.0.0.1:14550 --camera http://127.0.0.1:8554/stream.mjpg --model yolov8n.pt
```

---

## 📖 Belgeler
- **Mimarî Detaylar & Güvenlik Politikası:** [docs/architecture.md](docs/architecture.md)
- **TEKNOFEST 2026 Şartname Kriterleri:** [docs/SARTNAME_SABIT_KANAT_2026.md](docs/SARTNAME_SABIT_KANAT_2026.md)
- **Teknik Blueprint (AI Context):** [PROJECT_BLUEPRINT.md](PROJECT_BLUEPRINT.md)

---

## 📄 Lisans
Bu proje [MIT Lisansı](LICENSE) altında açık kaynak olarak lisanslanmıştır.
