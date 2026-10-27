# 2026 TÜBİTAK / TEKNOFEST İHA Yarışmaları — Sabit Kanat Kategorisi Şartları

> **Kaynak:** *İHA Yarışmaları Şartnamesi* (TR), **Versiyon V6 — 30.06.2026**  
> **Kapsam:** Yalnızca **Sabit Kanat** kategorisine ait tanım, teknik limit, görev ve puanlama özeti.  
> **Uyarı:** Bu belge ekip içi / AI bağlam özetidir. Çelişki halinde **resmi PDF şartname** geçerlidir.  
> IHAILOS yazılımı özellikle **Uluslararası İHA Yarışması Sabit Kanat — Görev 2** (görüntü işleme + isabetli yük bırakma) senaryosuna hizalanır.

---

## 1. Tanım

**Sabit Kanat İHA:** Taşıma kuvvetinin tamamını veya önemli bir bölümünü üzerindeki **hareket etmeyen kanat** sisteminden alan hava aracı.

- Sabit Kanat kategorisinde **yalnızca sabit kanat** İHA’lar yarışabilir.
- **DİKY (VTOL)** araçlar Sabit Kanat’a başvuramaz; yalnızca **Serbest Görev** kategorisinden başvurabilir.

---

## 2. Kategori Özeti (Şartname Madde 3.1)

Sabit Kanat takımları **iki görev** yapar (detay: Madde 10.1):

| Görev | Ne test edilir? |
|-------|-----------------|
| **1** | Otonom uçuş, silindirik muhafaza hacmi, kutudan çıkarıp hazırlama hızı, hafiflik, uçuş hızı, kararlılık, manevra (∞ / sonsuz) |
| **2** | Görev 1 isterleri + **yük taşıma** + **hedefe isabetli bırakma** |

### 2.1 Uluslararası vs Liseler — Görev 2 farkı

| | Uluslararası | Liseler Arası |
|--|--------------|---------------|
| Hedef konum | **Bilinmeyen** — görüntü işleme ile tespit zorunlu | **Önceden bilinen** koordinatlar |
| Hedefler / yük renkleri | **4×4 m mavi** hedefe **kırmızı** yük; **2×2 m kırmızı** hedefe **mavi** yük | **Ø10 m** hedefe **mavi** yük; **Ø2 m** hedefe **kırmızı** yük |
| Yük sayısı | **2 adet** zorunlu | **2 adet** zorunlu |

> IHAILOS: Uluslararası Görev 2 hattı → YOLO tespit, WGS84 geo-ref, balistik bırakma, hakem telemetrisi.

---

## 3. Teknik Limitler (Sabit Kanat)

### 3.1 Ağırlık

- **Toplam kalkış ağırlığı (yükler dahil) ≤ 4 kg.**

### 3.2 Tasarım / üretim

- Tasarım, boyutlandırma ve üretim **takım tarafından** yapılmalı.
- Hazır alınan İHA veya kopya tasarım → **elenme**.

### 3.3 Muhafaza kutusu (Sabit Kanat’a özel)

- **Silindirik** olmalı; kesit **daire** (elips değil).
- Tamamen kapalı; **1 kişi** taşıyabilir boyut/ağırlık.
- Yükler dahil (YKI / kumanda hariç) tüm İHA bileşenleri kutuda olmalı.
- Kutudan sarkan parça olmamalı; kulp vb. hacme dahil edilmez.
- Hacim hesabında **en dış ölçüler** esas alınır.
- Kutuda olmayan / sonradan eklenen harici malzeme → toplam puandan **0–5 ceza**.

### 3.4 Radyo / telemetri / emniyet

- RC: **uçtan uca şifreleme** zorunlu.
- Sinyal kaybında en fazla **5 s** içinde **fail-safe**.
- Sabit Kanat ve Döner Kanat’ta telemetri de **uçtan uca şifreli** olmalı.
- **Fail-safe (Sabit Kanat) konfigürasyonu:**
  - Gaz kesilmesi  
  - Tam yukarı irtifa (elevator)  
  - Tam sağ dümen  
  - Tam sağ eleron  
  - Flaplar tam aşağı  
- Kolay erişilebilir entegre **Power On/Off**; gücün **≤ 2 s** içinde kesilmesi.
- Arm/disarm: kumanda stick hareketleriyle.
- Jamming / spoofing / frekans müdahalesi → **elenme**.

### 3.5 İrtifa

- Alt irtifa limiti yok (yere temas etmeme kaydıyla).
- Üst irtifa: SHGM **AGL 120 m**.

### 3.6 Uçuş alanı (planlanan)

- Direkler arası uzunluk planı: **~150 m** (alan belli olunca resmi ölçü duyurulur).
- Takım başına en fazla **4 uçuş hakkı** (2 görev için); her görevde **en yüksek puanlı uçuş** sayılır.
- Uçuş alanına takımdan en fazla **4 üye**; danışman giremez.

---

## 4. Yük Kuralları (Madde 8.3.2 — Sabit Kanat için geçerli)

- Görev 1: **yüksüz** uçuş; puan yüksüz ağırlık üzerinden.
- Görev 2: **2 yük** zorunlu; özellikler (renk hariç) birbirinin aynı.
- Yük muhteviyatı/boyut takıma bırakılmış; ağırlıklar teknik kontrolde **önceden beyan**.
- İki farklı yük denemesi yapılabilir (ör. 800 g / 1200 g); **motor/pervane/ESC değiştirilemez**, yalnızca batarya değişimine izin.
- Yüke elektronik aktif kontrol **yasak**; paraşüt benzeri pasif sistemler aynı renkte olabilir.
- Yük yere düşünce **parçalanmamalı**; bütünlük kaybı → mesafe ölçülmez, isabet puanı yok.
- Yüke işaret / şerit / karekod eklenemez (yalnız belirtilen renk).

### 4.1 İsabet ölçümü (Sabit Kanat)

- Ölçülen nokta: düştüğü yer değil, **en son durduğu yer**.
- Merkezden mesafe; Sabit Kanat için ölçüm penceresi **≤ 20 m**.
- 20 m dışı → o kriterden **0 puan**.

### 4.2 Görüntü işleme (Uluslararası)

- Hedefler 2. görev uçuşu öncesi **rastgele** konumlanır (dönüş alanına yakın değil).
- Görüntü işleme **kanıtlanmalı** (YKI üzerinden hakeme); işlemesiz isabet → isabet puanı **yok**.
- Yardımcı bilgisayara entegre **tek kamera**; ikinci tarama kamerası **yasak**.
- Sabit kanat, döner kanat hedeflerini (**altıgen / üçgen**) **algılamamalı**.
- Hedef tespiti aşamasında: **2. direği dışarıdan aldıktan sonra** tespit zorunlu.

---

## 5. Süre Limitleri

| Aşama | Azami süre |
|--------|------------|
| Kurulum | **8 dk** (hakem “başla” → pilot “hazırız”) |
| Uçuşa hazırlık (GPS, arm vb.) | **8 dk** |
| Görev 1 uçuş | **5 dk** |
| Görev 2 uçuş | **10 dk** |

- Kutudan çıkarma / hazırlıkta en fazla **3 üye** (+ YKI için 1 üye daha).
- Danışman hazırlığa yardım edemez.
- Hazırlıkta güç ver/kes dışında her ek müdahale: **(müdahale süresi + 10 s)** kurulum süresine eklenir.
- Uçuş süresi: başlangıç/bitiş çizgisini geçişle başlar/biter; çizgi sonrası kırım → uçuş başarısız.
- Kutuda güç sistemi önceden **faal olmamalı**.

---

## 6. Otonomi Katsayısı `c`

Puan formüllerinde çarpan:

| Durum | Uluslararası | Liseler |
|--------|--------------|---------|
| Manuel kalkış + manuel uçuş + manuel iniş | — | **0,5** |
| Manuel kalkış ve/veya manuel iniş, otonom uçuş | **0,7** | **0,7** |
| Otonom kalkış + otonom uçuş + otonom iniş | **1,0** | **1,0** |

---

## 7. Görev 1 — Manevra (∞ / Sonsuz)

Ortak (Uluslararası + Liseler):

1. Kalkış çizgisinden havalan.
2. Direk **2**’ye git; **2’yi dışarıdan** al.
3. Direk 1–2 arasında **∞ (yatay 8)** manevrası — **art arda 2 tur**.
4. Direk 2 arkasından dış dönüş; başlangıç/bitiş çizgisini geçerek bitir.
5. İniş: 1. ve 2. direk arası uygun alan (takım seçer).
6. Manevra yapılamazsa Görev 1 = **0**; Görev 2 hakkı **yok**.
7. Direği **içeriden** alma (dışından dönmeme): 1. kez **−10** ceza; 2. kez uçuş **başarısız**.

---

## 8. Görev 2 — Özet Akış

### 8.1 Uluslararası (IHAILOS hedefi)

1. Silindirik kutudan kurulum + otonom uçuş.
2. Konumu **bilinmeyen** iki hedefi **görüntü işleme** ile bul.
3. Her hedefe **bir yük** bırak (renk eşleşmesi yukarıdaki tablo).
4. Direk 2 dışarıdan alındıktan sonra hedef tespiti.
5. Uçuş ≤ 10 dk; isabet ölçümü ≤ 20 m penceresi.

### 8.2 Liseler

1. Aynı kurulum/otonomi çerçevesi.
2. Konumu **bilinen** Ø10 m ve Ø2 m hedeflere birer yük.
3. Hedef renkleri liselerde ayrıca belirtilmemiş (konum verildiği için).

---

## 9. Puanlama (Sabit Kanat)

### 9.1 Genel tablo

**Uluslararası Sabit Kanat (Tablo 2):**

| Kalem | Puan |
|--------|------|
| Hazırlık ve Görev Videosu | 10 |
| Görev 1 | 30 |
| Görev 2 | 70 |
| **Toplam (ceza hariç)** | **110** |
| Ceza 1 (centilmenlik / kutu dışı malzeme) | 0–5 |
| Ceza 2 (direk içten alma) | 0–10 |

**Liseler Sabit Kanat (Tablo 4):** Video 10 + G1 30 + G2 70 = **110** (ceza 0–5 / 0–10).

> Görev 2’yi başarıyla tamamlayamayan takım **Derece Ödülü** alamaz. Görev 1 başarısı Görev 2 ön şartıdır.

### 9.2 Formüller (özet)

**Görev 1 (Sabit Kanat — Uluslararası / Liseler ortak yapı):**

\[
P_{G1} = c \cdot \Big[
6\cdot\frac{Vol_{min}}{Vol_{tk}}
+ 6\cdot\frac{t_{min,haz}}{t_{tk,haz}}
+ 6\cdot\frac{W_{0,min}}{W_{0,tk}}
+ 12\cdot\frac{t_{min,uçuş}}{t_{tk,uçuş}}
\Big] - \text{Ceza}
\]

**Görev 2 — Uluslararası Sabit Kanat:**

\[
P_{G2} = c \cdot \Big[
4\cdot\frac{Vol_{min}}{Vol_{tk}}
+ 4\cdot\frac{t_{min,haz}}{t_{tk,haz}}
+ 4\cdot\frac{W_{0,min}}{W_{0,tk}}
+ 20\cdot\frac{W_{yük,tk}}{W_{yük,max}}
+ 10\cdot\frac{d_{4\times4,min}}{d_{4\times4,tk}}
+ 20\cdot\frac{d_{2\times2,min}}{d_{2\times2,tk}}
+ 8\cdot\frac{t_{min,uçuş}}{t_{tk,uçuş}}
\Big] - \text{Ceza}
\]

**Görev 2 — Liseler Sabit Kanat:**

\[
P_{G2} = c \cdot \Big[
4\cdot\frac{Vol_{min}}{Vol_{tk}}
+ 4\cdot\frac{t_{min,haz}}{t_{tk,haz}}
+ 4\cdot\frac{W_{0,min}}{W_{0,tk}}
+ 20\cdot\frac{W_{yük,tk}}{W_{yük,max}}
+ 10\cdot\frac{d_{10m,min}}{d_{10m,tk}}
+ 20\cdot\frac{d_{2m,min}}{d_{2m,tk}}
+ 8\cdot\frac{t_{min,uçuş}}{t_{tk,uçuş}}
\Big] - \text{Ceza}
\]

**Parametre anlamı (Sabit Kanat):**

| Sembol | Anlam |
|--------|--------|
| \(Vol\) | Muhafaza kutusu hacmi (küçük daha iyi) |
| \(t_{haz}\) | Kurulum / hazırlık süresi |
| \(W_0\) | Yüksüz ağırlık |
| \(W_{yük}\) | Taşınan yük ağırlığı (büyük daha iyi) |
| \(d_{4\times4}\), \(d_{2\times2}\) | Uluslararası hedef merkez uzaklıkları |
| \(d_{10m}\), \(d_{2m}\) | Liseler hedef merkez uzaklıkları |
| \(t_{uçuş}\) | Görev uçuş süresi |

---

## 10. IHAILOS Yazılımına Doğrudan Yansıyan İsterler

| Şartname isteri | IHAILOS karşılığı |
|-----------------|-------------------|
| Görüntü ile hedef tespiti (Uluslararası G2) | `vision_engine` YOLO + debounce |
| Hedef WGS84 konumu | `estimator/geo_referencing.py` |
| İsabetli bırakma / rüzgar | `estimator/ballistics.py` |
| Otonom uçuş + emniyet | `mission_coordinator` FSM, RC override, \|R\|/\|P\|&lt;5° |
| Servo ile bırakma | MAVLink AUX9 PWM 2000 |
| Hakem / kanıt telemetrisi | `comms_gateway/referee_client.py` |
| ≤20 m isabet penceresi | Görev değerlendirme / skorlama hedefi (yazılımda takip edilmeli) |
| Tek görev kamerası | Mimari varsayım: tek vision hattı |
| AGL ≤ 120 m | FSM kapısı: taze aşağı yönlü `DISTANCE_SENSOR` AGL ister; yok/eski/>120 m ise engagement kapalı. `relative_alt_m` AGL kabul edilmez. |
| MTOW ≤ 4 kg | Platform kısıtı (yazılım dışı ama görev tasarımında) |

**Donanım doğrulama sınırı:** RC ve telemetri şifrelemesi, ≤5 s radyo failsafe, sabit kanat failsafe yüzey konfigürasyonu ve ≤2 s güç kesme yazılım FSM'siyle kanıtlanamaz; teknik kontrolden önce FC/alıcı üzerinde ayrıca test edilmelidir.

---

## 11. Hızlı Kontrol Listesi (Sabit Kanat)

- [ ] MTOW ≤ 4 kg (yüklü)
- [ ] Silindirik (daire kesit) muhafaza kutusu
- [ ] Uçtan uca şifreli RC + telemetri
- [ ] Fail-safe sabit kanat profili doğrulandı
- [ ] Power cut ≤ 2 s
- [ ] Görev 1: yüksüz ∞ iki tur, direk dıştan
- [ ] Görev 2: 2 yük, renk/hedef eşlemesi doğru
- [ ] Uluslararası: CV tespiti kanıtlı; rastgele hedef; direk-2 sonrası tarama
- [ ] İsabet ölçümü ≤ 20 m penceresi
- [ ] Süreler: kurulum/hazırlık 8+8; G1≤5 dk; G2≤10 dk
- [ ] İrtifa ≤ AGL 120 m

---

## 12. Belge Bakımı

| Alan | Değer |
|------|--------|
| Resmi sürüm | V6 (30.06.2026) |
| Özet dosyası | `docs/SARTNAME_SABIT_KANAT_2026.md` |
| Proje blueprint | `PROJECT_BLUEPRINT.md` |
| Güncelleme | Yeni şartname sürümünde bu özet yeniden üretilmeli |

*Şekiller (uçuş güzergâhı Şekil 2–5 vb.) için resmi PDF’e bakınız.*
