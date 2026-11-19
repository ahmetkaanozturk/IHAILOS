# IHAILOS — Şartname Uygunluk Raporu ve 5 Kişilik Görev Planı

> **UYARI (2026-11-19):** Bu belge **TÜBİTAK İHA Yarışması – Sabit Kanat** şartnamesine göre yazılmıştır. Takımın katıldığı yarışma bu değil, TUSAŞ'ın **İleri Otonom Sistemler Tasarım ve Operasyon Yarışması**'dır. Buradaki görev planı o yarışma için **geçerli değildir**. Yalnız bölüm 3'teki kod hataları (H1–H11), bu kod yeniden kullanılacaksa hâlâ geçerlidir.

Tarih: 2026-11-19 · Kapsam: `IHAILOS` (otonomi) + `IHASIMULASYON` (Gazebo/SITL)

**Karşılaştırılan şartname:** TÜBİTAK İHA Yarışmaları Şartnamesi **V7 (06.09.2026)**, resmi İngilizce PDF, Uluslararası — Sabit Kanat.
Repodaki özet (`docs/SARTNAME_SABIT_KANAT_2026.md`) V6'ya göre yazılmış. 2026 yarışması 8–13 Eylül 2026'da bitti; teknofest.org'da henüz 2027 şartnamesi yok. Bu plan V7'ye göredir; 2027 V1 çıkınca fark kontrolü yapılmalı (görev K2).

---

## 1. Kısa durum

Mevcut kod, **düz uçan bir uçakta tek hedefe tek yük bırakma** fikrinin sentetik prototipidir. Matematik çekirdeği (piksel→WGS84, Kalman, serbest düşüş) yazılmış ve 25 birim testi geçiyor. Ama yarışma görevini uçtan uca yapacak parçaların çoğu yok ve bazı kısımlar başka bir yarışmanın (Savaşan İHA: "hakem sunucusu", "HSS", "kilitlenme") kavramlarıyla yazılmış.

Bu haliyle sistem **Görev 1'i yapamaz**; Görev 1 yapılmadan Görev 2 hakkı doğmaz.

---

## 2. Şartname isterleri — durum tablosu

| # | Şartname isteri (V7) | Koddaki durum | Sonuç |
|---|---|---|---|
| 1 | Görev 1: direk 1–2 çevresinde ∞ manevrası, art arda 2 tur, direkler dışarıdan, sonra direk 2 dışından dönüp bitiş çizgisi; ≤ 5 dk | Hiç kod yok (rota üretimi, görev yükleme, tur sayımı yok) | **YOK** |
| 2 | Otonom kalkış + uçuş + iniş → katsayı c = 1,0 (yoksa 0,7) | Kalkış, iniş, arm, görev yükleme komutu yok; FSM uçağın zaten havada olduğunu varsayıyor | **YOK** |
| 3 | Görev 2: konumu bilinmeyen **2 hedef**, **2 yük** | FSM tek hedefe tek bırakma yapıp RTL'e geçiyor | **EKSİK** |
| 4 | Kırmızı yük → 4×4 m mavi hedef; mavi yük → 2×2 m kırmızı hedef | Renk/hedef eşlemesi yok; yalnız AUX9 tetikleniyor, AUX10 hiç kullanılmıyor | **YOK** |
| 5 | Görüntü işleme ile tespit | Sınıflar `HSS/HOSTILE/FRIENDLY/VEHICLE/BUILDING`; mavi kare / kırmızı kare sınıfı yok. Eğitilmiş model, veri seti, eğitim betiği yok (`yolov8n.pt` genel amaçlı modeldir, bu hedefleri tanımaz) | **YOK** |
| 6 | Altıgen ve üçgen (döner kanat hedefleri) algılanmamalı | Negatif filtre yok; simde modelleri var ama test yok | **YOK** |
| 7 | Tespit, direk 2 dışarıdan alındıktan sonra yapılmalı | Böyle bir kapı yok | **YOK** |
| 8 | Tarama: hedefler 30×100 m alanda rastgele | `SEARCH` durumu hiç rota komutu göndermiyor (pasif bekliyor) | **YOK** |
| 9 | Tespit edilen hedefin **etiketli görüntüsü YKİ üzerinden hakeme anında** gösterilmeli; gösterilemezse isabet puanı yok | YKİ ekranı yok. Bunun yerine şartnamede olmayan bir "hakem sunucusu"na HTTP gönderimi var | **YOK / yanlış hedef** |
| 10 | Uçuş alanı 200×300 m, tampon bölge yok | Geofence sabit kodlu ~3,4×4,4 km Ankara kutusu; yaklaşma noktası bırakmadan 300 m geride + 75 m yarı genişlik → bu alana sığmaz | **UYUMSUZ** |
| 11 | Süre bitiş çizgisinden geçince durur; iniş direk 1–2 arası | Bırakma sonrası RTL; bitiş çizgisi ve iniş mantığı yok | **YOK** |
| 12 | İsabet ölçümü hedef merkezinden ≤ 20 m; dışı 0 puan | Bırakma ~20 m erken tetikleniyor (bkz. H2); sürükleme ve rüzgar bırakma noktasına uygulanmıyor | **RİSKLİ** |
| 13 | Tek kamera | Tek görüntü hattı | UYGUN |
| 14 | AGL ≤ 120 m | Emniyet kapısında var; ama balistik planda AGL yerine `relative_alt_m` kullanılıyor (kendi kuralına aykırı) | KISMEN |
| 15 | Failsafe yüzey konfigürasyonu (gaz kes, tam yukarı irtifa, tam sağ dümen, tam sağ eleron, flap aşağı), ≤ 5 s | Yalnız SITL'de `THR_FAILSAFE`/`FS_LONG_TIMEOUT` var; yüzey konfigürasyonu tanımlı değil. Ayrıca yazılım failsafe'i ezebiliyor (bkz. H1) | **EKSİK** |
| 16 | RC ve telemetri uçtan uca şifreli; güç kesme ≤ 2 s; arm/disarm stick ile | Donanım işi; yazılımda kanıt/kontrol listesi yok | Donanım |
| 17 | Hakem uçuşu sonlandırırsa failsafe + güvenli iniş | Yazılımda "pilot devraldıysa hiçbir komut gönderme" garantisi yok | **EKSİK** |
| 18 | Hazırlık ve Görev Videosu: YKİ görüntüsü doğrudan ekran kaydı olmalı, uçuş kesintisiz | YKİ olmadığı için kayıt da yok | **YOK** |
| 19 | Kurulum 8 dk + hazırlık 8 dk; süre puanı etkiler | Yazılımın hızlı/otomatik açılışı, ön uçuş kontrolü yok | **YOK** |

---

## 3. Doğrulanmış hatalar (deneyle gösterildi)

Aşağıdakiler kod okunarak bulundu ve geçici bir betikle çalıştırılarak doğrulandı (repo dosyaları değiştirilmedi).

| # | Hata | Kanıt | Yer |
|---|---|---|---|
| **H1** | **Pilot kontrolü alınca yazılım uçağı GUIDED moda geri zorluyor.** Pilot FBWA/MANUAL'e geçince FSM bunu "güvenlik ihlali" sayıp `GO_AROUND`'a giriyor ve `set_mode_guided()` gönderiyor. Aynı yol failsafe modunda da çalışır. | `TRACK_LOCK` iken FBWA → 1 adet `DO_SET_MODE` (mod 15 = GUIDED) gönderildi | `state_machine.py:175-185` |
| **H2** | **Yük ~20 m erken bırakılıyor.** Bırakma, planlanan noktaya 25 m yarıçapla girilince tetikleniyor. | 25 m/s, 10 Hz düz yaklaşmada servo komutu noktadan 20,2 m önce gitti. Ölçüm penceresi 20 m | `state_machine.py:320-326`, `release_radius_m = 25` |
| **H3** | **Dönüşte görev iptal oluyor.** \|yatış\| < 5° kapısı `TRACK_LOCK`, `APPROACH_PLAN`, `FINAL_RUN` boyunca zorunlu; sabit kanat yaklaşma noktasına dönerken 5°'yi aşar → sonsuz `GO_AROUND` döngüsü. | `TRACK_LOCK` iken 20° yatış → `GO_AROUND` | `state_machine.py:219,243,301` |
| H4 | Yalnız tek servo (9) tetikleniyor, sonra RTL | Deneyde gönderilen kanal: `[9]`, son durum `VERIFY` | `state_machine.py:375-383` |
| H5 | Aynı kamera karesi Kalman'a tekrar tekrar giriyor (kestirim 10 Hz, görüntü 5 Hz, kare numarası kontrolü yok; `dt` sabit 0,05) → filtre gerçekte olmayan ölçümlerle "yakınsamış" görünür | Kod okuma | `main.py:255,343` |
| H6 | Kare, çekildiği andaki değil en son telemetriyle eşleniyor (`sync_telemetry_seq` kaydediliyor ama kullanılmıyor) → konum hatası | Kod okuma | `main.py:235-254` |
| H7 | Sunucu yokken her telemetri paketi (5 Hz) diske kuyruklanıyor; kuyruk sınırsız büyüyor | Kod okuma | `referee_client.py:409-418` |
| H8 | Rüzgar sürüklenmesi hesaplanıyor ama bırakma noktasına uygulanmıyor; hava direnci yok | Kod okuma | `ballistics.py:231-238` |
| H9 | Simde yükler yalnız görsel; ayrılabilir yük yok, servo 9/10 simde bir şeye bağlı değil → bırakma simde test edilemez | Kod okuma | `model.sdf:137-160` |
| H10 | Sim dünyası 200×300 m alana uymuyor (hedefler X = 280–320 m, tarama çerçevesi 120×140 m); model kütlesi 3,25 kg, belgede 3,40 kg | Kod okuma | `teknofest_2026.world`, `model.sdf:37` |
| H11 | İki klasör de git deposu değil (`.git` yok) — 5 kişi ortak çalışamaz | `git status` | — |

**Tasarım riski (karar gerekir):** Kamera ufkun 30° altına bakıyor. 80 m irtifada görüntü merkezi ~139 m ileriye düşer; 2×2 m hedef yaklaşık 14 piksel genişliğinde kalır ve 1° tutum hatası yerde ~5,6 m konum hatası yapar. 1 m'lik üçgeni 2 m'lik kareden ayırmak bu çözünürlükte zordur. Daha dik (aşağı bakan) kamera ve daha alçak tarama irtifası değerlendirilmeli (görev V5).

---

## 4. "Sistem bitti" tanımı

Tüm görevler bittiğinde aşağıdakiler kanıtıyla sağlanmalı:

1. **Görev 1 (SITL):** tek komutla otonom kalkış → 2 tur ∞ → bitiş çizgisi → otonom iniş. Art arda 10 koşunun 10'unda direk ihlali 0, süre < 5 dk.
2. **Görev 2 (SITL):** hedefler her koşuda rastgele; 2 yük doğru renk eşleşmesiyle bırakılıyor. Art arda 20 koşuda: yanlış eşleşme 0, altıgen/üçgen tespiti 0, yüklerin en az %90'ı merkeze ≤ 5 m, süre < 10 dk.
3. **Emniyet:** pilot mod değiştirdiğinde veya failsafe'te yazılım **hiçbir** komut göndermiyor (otomatik test + yerde verici kapatma testi).
4. **Hakem kanıtı:** YKİ ekranında tespit anında etiketli görüntü görünüyor ve kaydı alınıyor.
5. **Gerçek uçuş:** aynı senaryolar gerçek uçakta en az 3'er kez başarıyla uçuldu; atış dağılımı ölçüldü.
6. **Kurulum:** yazılım güç verildikten sonra elle müdahale olmadan hazır; ön uçuş kontrolü tek ekranda.

---

## 5. Görev listesi (5 kişi)

Her görevin sonunda bir **kabul ölçütü** var; ölçüt sağlanmadan görev bitmiş sayılmaz.

### Faz 0 — İlk hafta, herkes (diğer her şeyden önce)

| Kod | Görev | Sahibi | Kabul |
|---|---|---|---|
| F0-1 | Git deposu kur (2 repo), dal kuralı, her değişiklikte otomatik `pytest` | Üye 1 | 5 üye de değişiklik gönderebiliyor; testler otomatik koşuyor |
| F0-2 | H1 düzelt: mod AUTO/GUIDED değilken yazılım hiçbir MAVLink komutu göndermesin | Üye 2 | Test: her durumda FBWA/MANUAL/RTL telemetrisi verilince gönderilen komut sayısı 0 |
| F0-3 | H2 + H3 düzelt (bırakma zamanlaması, yatış kapısı yalnız son düz hatta) | Üye 4 | Test: tetikleme hatası < 2 m; 30° yatışla dönüş `GO_AROUND` üretmiyor |

### Üye 1 — Kaptan: entegrasyon, görev yöneticisi, kabul

| Kod | Görev | Kabul |
|---|---|---|
| K1 | Tek ayar dosyası: bitiş çizgisi, direk 1/2, 200×300 m alan, tarama alanı, kamera açısı, servo kanalları, yük kütlesi. `main.py`'deki sabit Ankara değerlerini kaldır | Koordinat değiştirmek için kod değişikliği gerekmiyor |
| K2 | Şartname özetini V7'ye güncelle; 2027 V1 çıkınca fark listesi çıkar ve görevlere yansıt | Özet belgesi V7; 2027 farkları 1 hafta içinde listelendi |
| K3 | Üst görev yöneticisi: `ÖN KONTROL → KALKIŞ → (G1: ∞×2) / (G2: DİREK 2 → TARAMA → [YAKLAŞ → BIRAK] × 2) → BİTİŞ ÇİZGİSİ → İNİŞ`. Mevcut FSM'i bunun alt parçası yap | SITL'de G1 ve G2 tek komutla baştan sona koşuyor |
| K4 | Haftalık entegrasyon koşusu ve sonuç tablosu (Üye 5'in G5 aracıyla) | Her hafta tabloya 1 satır: geçen/kalan, mesafe, süre |
| K5 | Rapor ve video için yazılım bölümleri (mimari, test kanıtları) | PSR ve video teslimlerinden 2 hafta önce hazır |

### Üye 2 — Uçuş kontrol ve seyrüsefer (ArduPlane / MAVLink)

| Kod | Görev | Kabul |
|---|---|---|
| N1 | MAVLink katmanı: arm, mod, görev yükleme, görev ilerleme takibi; her komutta onay (ACK) kontrolü ve tekrar deneme | Reddedilen komut loglanıyor; onaysız komut "başarılı" sayılmıyor |
| N2 | Otonom kalkış ve otonom iniş (iniş direk 1–2 arası alana) | SITL 10/10; pilot girdisi 0 |
| N3 | Görev 1 rota üretici: direk ve çizgi koordinatlarından ∞ rotası, 2 tur, direkler dışarıdan | Otomatik denetleyici: ihlal 0, 2 tur, çizgi geçildi, süre < 5 dk; tur süresi raporlanıyor |
| N4 | Görev 2 rotaları: direk 2'yi dışarıdan al → 30×100 m tarama geçişleri → hedef başına yaklaşma hattı (200×300 m alana sığan; 300 m geri çekilme yerine yeni geometri) | Tüm rota noktaları alan poligonu içinde; tarama şeridi tüm alanı kapsıyor |
| N5 | Görev sonu: bitiş çizgisinden geçiş + iniş (RTL yerine) | Süre çizgide duruyor; iniş hasarsız |
| N6 | Otopilot parametreleri: şartnamedeki failsafe yüzey konfigürasyonu, ≤ 5 s; otopilot içi alan sınırı ve 120 m tavan | Yerde verici kapatma testi videosu: 5 s içinde doğru yüzey konumları |
| N7 | Uçuş hızı ayarı (G1'de 12 puan, G2'de 8 puan uçuş süresinden) | Tur süresi önceki haftaya göre ölçülüp iyileştiriliyor |

### Üye 3 — Görüntü işleme

| Kod | Görev | Kabul |
|---|---|---|
| V1 | Sınıf şemasını değiştir: `mavi_kare_4x4`, `kirmizi_kare_2x2` (+ negatif: altıgen, üçgen). `HSS/HOSTILE…` kaldır | Veri sözleşmesi ve testler yeni sınıflarla geçiyor |
| V2 | Veri seti: simden otomatik etiketli + gerçek çekim (branda hedefler; farklı ışık, irtifa, açı) | Eğitim/doğrulama/test ayrımı; test seti gerçek çekimden |
| V3 | Model eğitimi + ikinci aşama şekil doğrulama (renk maskesi, köşe sayısı, en-boy) | Test setinde: kare hedeflerde yakalama ≥ %95; altıgen/üçgende yanlış tespit 0 |
| V4 | Çoklu hedef: doğrulama (debounce) hedef başına ayrı; şu an yalnız en yüksek güvenli tek kutu izleniyor | Aynı karede 2 hedef + 2 negatif varken iki hedef de ayrı ayrı kilitleniyor |
| V5 | Kamera kararı ve kalibrasyon: montaj açısı (30° ileri mi, aşağı bakan mı), iç parametre + distorsiyon | Karar notu (piksel boyutu ve konum hatası hesabıyla); kalibrasyon dosyası depoda |
| V6 | Uçak bilgisayarında hız: TensorRT, kare zaman damgası, gecikme ölçümü | ≥ 15 kare/s; uçtan uca gecikme ölçülüp ayar dosyasına yazıldı |
| V7 | "Direk 2 alındı" bayrağı gelmeden tespitleri geçersiz say | Test: bayraktan önceki tespit hedef üretmiyor |

### Üye 4 — Kestirim, balistik ve yük bırakma

| Kod | Görev | Kabul |
|---|---|---|
| E1 | Zaman eşlemesi: kareyi çekildiği andaki telemetriyle eşle (ara değerleme); her kare filtreye bir kez girsin; gerçek `dt` (H5, H6) | Simde bilinen hedef konumuna göre hata tarama irtifasında < 3 m |
| E2 | İki hedef için ayrı izleyici (hedef sabit → sabit konum modeli), çok geçişli ortalama, belirsizlik çıktısı | İki hedef karışmıyor; belirsizlik eşiği aşılmadan bırakma izni yok |
| E3 | Balistik model: hava direnci + rüzgar + bırakma gecikmesi (servo, mekanizma, yazılım); AGL kullan | Simde rüzgarlı koşuda (25 km/sa'e kadar; şartnamedeki rüzgar eşiği) düşüş noktası tahmini hatası < 3 m |
| E4 | Bırakma tetikleme: yarıçap yerine "bırakma hattını geçme anı"; yüksek hızlı döngü | Tetikleme hatası < 2 m (F0-3'ün kalıcı hali) |
| E5 | İki yük: AUX9 kırmızı → mavi 4×4, AUX10 mavi → kırmızı 2×2; yanlış eşleşmeyi engelleyen kilit; aynı yükü iki kez bırakmama | Test: yanlış kanal komutu hiçbir senaryoda üretilmiyor |
| E6 | Emniyet kapılarını yeniden tasarla: hangi kapı hangi aşamada zorunlu (yatış yalnız son düz hatta) | Kapı tablosu belgede; her satırın testi var |
| E7 | Saha atış testleri: gerçek yükle en az 20 atış, dağılım ölçümü, model kalibrasyonu | Atışların %90'ı merkeze ≤ 5 m; sonuç tablosu depoda |

### Üye 5 — YKİ, hakem kanıtı, simülasyon ve test altyapısı

| Kod | Görev | Kabul |
|---|---|---|
| G1 | YKİ ekranı: canlı etiketli görüntü (kutu, sınıf, hedef koordinatı, saat); tespit anı kareleri ekranda sabitlenir ve kaydedilir | Hakeme gösterilecek ekran tek pencerede; tespitten ≤ 2 s sonra görünüyor |
| G2 | Uçak → yer görüntü/veri hattı; bağlantı kopunca uçakta kayıt, gelince gönderim. "Hakem sunucusu" modülünü buna dönüştür; telemetriyi diske yığma (H7) | Bağlantı 30 s kesilip gelince tespit kareleri YKİ'ye ulaşıyor; disk kullanımı sınırlı |
| G3 | Uçuş kaydı ve tekrar oynatma: tüm paketler + video | Bir uçuş kaydı masada baştan sona tekrar oynatılabiliyor |
| G4 | Simülasyon düzeltmeleri: 200×300 m alan, 30×100 m tarama alanı, her koşuda rastgele hedef, ayrılabilir yükler (AUX9/10), rüzgar (H9, H10) | Simde bırakılan yük yere düşüyor ve konumu ölçülebiliyor |
| G5 | Otomatik senaryo koşucu + puanlayıcı: tek komutla G1/G2; çıktı: direk ihlali, süre, yük–merkez mesafesi, puan tahmini | `K4` tablosu bu araçla üretiliyor |
| G6 | Video altyapısı: YKİ ekran kaydı, üçlü ekran düzeni (uçak, pilot, YKİ) | Deneme videosu şartname düzeninde |
| G7 | Hızlı kurulum: güç verilince yazılım kendiliğinden açılır; ön uçuş kontrol ekranı (GPS, kamera, model, servo, telemetri) | Güç verildikten sonra elle komut yok; kontrol ekranı tümü yeşil olmadan kalkış izni yok |

---

## 6. Sıra ve bağımlılıklar

2027 takvimi henüz yok. 2026'da rapor Mart, video Temmuz, final Eylül'deydi; aşağıdaki sıra buna göre önerilmiştir.

| Faz | İçerik | Bitiş hedefi |
|---|---|---|
| 0 | F0-1…3, K1, V1 | 1. hafta |
| 1 — Görev 1 simde | N1, N2, N3, N5, G4 (alan), G5 | Aralık sonu |
| 2 — Görev 2 simde | K3, N4, V2–V4, V7, E1–E6, G1, G4 (yük) | Şubat sonu |
| 3 — Donanım | V5, V6, N6, N7, G2, G3, G7; ilk gerçek uçuşlar | Nisan sonu |
| 4 — Saha | E7, tekrar uçuşlar, G6, K5 | Video tesliminden 3 hafta önce |

Kritik zincir: **F0 → N1/N2 → N3 (Görev 1) → K3 → N4 + V3 + E3/E4 → E7.** Görüntü işleme (Üye 3) ve simülasyon (Üye 5) baştan itibaren paralel yürür.

---

## 7. Kaptanın vermesi gereken kararlar

1. Hedef 2027 yarışması mı? (Şartname çıkana kadar V7 esas alınır.)
2. Donanım: uçak, otopilot, yardımcı bilgisayar ve kamera belli mi? (V5, V6, N6 buna bağlı.)
3. Kamera açısı: 30° ileri bakış mı, aşağı bakan mı? (Bölüm 3 sonundaki risk.)
4. "Hakem sunucusu" modülü kaldırılıp YKİ'ye dönüştürülsün mü? (Öneri: evet; şartnamede sunucu yok.)
