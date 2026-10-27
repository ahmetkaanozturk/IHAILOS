# Project Rules: TEKNOFEST Sabit Kanat Otonom Görev Sistemi
- Hedef Ortam: Ubuntu 22.04 LTS, ROS 2 Humble, Python 3.10, ArduPlane (MAVLink).
- Kodlama Standartları: Asla "TODO", "pass" veya yarım bırakılmış (placeholder) fonksiyon yazma. Her matematiksel formülü (Geo-referencing, Balistik, Kalman) tam ve çalışır şekilde NumPy/SciPy ile kodla.
- Mimari: Sistem 4 ana modülden oluşur: 
  1. `vision_engine` (YOLO + TensorRT + Timestamp)
  2. `estimator` (Geo-referencing WGS84 + Kalman Filter + Wind/Ballistic math)
  3. `mission_coordinator` (Finite State Machine + Shapely Geofence + MAVLink commands)
  4. `comms_gateway` (Referee Server TCP/UDP/HTTP + Local Queue Buffer)
- Güvenlik Kuralı: Her kontrol döngüsünde RC Override (flight_mode not in ['AUTO', 'GUIDED']) ve |Roll| < 5, |Pitch| < 5 kontrolü zorunludur.
- Görev salımı için telemetri, HEARTBEAT, konum, tutum ve aşağı bakan rangefinder ölçümleri taze olmalıdır. AGL kaynağı yalnız gövdeye dikey aşağı monte edilmiş `DISTANCE_SENSOR` olarak kabul edilir; ölçüm yoksa/eskiyse veya AGL > 120 m ise `engage_authorized` kapalı kalır. `relative_alt_m` AGL yerine kullanılamaz. Montaj/yön ve ölçüm doğruluğu teknik kontrolden önce sahada doğrulanmalıdır.
- Fail-safe yüzey konfigürasyonu, 5 saniye sınırı, RC/telemetri şifrelemesi ve 2 saniye güç kesme donanım/uçuş kontrolcüsü doğrulamasıdır; bu Python kontrolleri bunların yerine geçmez.
- Tip Kontrolleri: Tüm veri sözleşmelerinde (Data Contracts) Python `dataclass` veya `Pydantic` ve `type hinting` kullan.