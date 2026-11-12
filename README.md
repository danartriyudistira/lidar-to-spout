# Lidar To Spout

Visualisasi scanner LiDAR YDLIDAR X3 untuk **interactive floor**, dikirim ke Resolume / TouchDesigner
sebagai video **Spout**, lengkap dengan **motion mask** dan **OSC per kaki**.

```
YDLIDAR X3 (COM4, 115200) → parser AA55 → crop zona → background filter → deteksi kaki
                          → render 3840×1080 → Spout "Lidar Beauty"
                                            → Spout "Lidar Mask"
                                            → OSC 127.0.0.1:9000
```

---

## 1. Instalasi

Butuh **Python 3.11 (64-bit)** — sudah terpasang di mesin ini.

```bat
cd "C:\Users\holut\OneDrive\Desktop\Lidar To Spout"
python -m pip install -r requirements.txt
```

Dependency: `pyserial`, `SpoutGL` (Spout SDK + Spout.dll), `moderngl`, `numpy`, `glfw`,
`PyOpenGL`, `imgui`, `pillow`.

## 2. Menjalankan

```bat
run.bat
```

atau langsung:

```bat
python src\app.py
```

Tanpa hardware (pakai rekaman demo sintetis):

```bat
run_replay.bat
```

Rekaman demo (`data/demo_room.bin`) adalah ruangan sintetis 8×4 m dengan dua "orang"
berjalan — dibuat oleh `tools/make_demo.py`. Untuk merekam scan asli:

```bat
python tools\record.py --seconds 10 --out data\scan_saya.bin
python src\app.py --replay data\scan_saya.bin
```

Opsi penting:

| Opsi | Arti |
|---|---|
| `--port COM4` | ganti port serial |
| `--baud 115200` | baud rate |
| `--replay data\demo_room.bin --speed 1` | putar rekaman, tidak memegang COM4 |
| `--headless` | tanpa window (Spout tetap jalan, hemat GPU) |
| `--no-spout` / `--no-osc` / `--no-tuio` | matikan output tertentu |
| `--width 3840 --height 1080 --fps 60` | ukuran kanvas & target fps |
| `--snapshot out.png --frames 300` | simpan frame PNG (bonus: `out_mask.png`) lalu keluar |
| `--debug` | log statistik tiap 30 frame |

Hotkey: `H` panel, `P` preview, `Q` edit quad, `A` unlock vertex area, `S` simpan config,
`Del` hapus area, `Ctrl+D` duplikat area, panah geser area, `Esc` batal/keluar.

## 3. Setup fisik LiDAR

- X3 dipasang **8–15 cm dari lantai**, menghadap menyapu permukaan lantai.
- Jangan menempel di lantai: terlalu rendah → hanya kena ujung sepatu, track mudah hilang.
- Jangan menyorotkan proyektor langsung ke lensa LiDAR.
- Posisi ideal: **di tengah area interaktif**, supaya semua kaki berada di radius < 5 m
  (jangkauan efektif X3 di lantai ±5 m, spesifikasi 8 m hanya untuk target reflektif).
- Kecepatan motor X3 di mesin ini terbaca ±11 Hz (latensi ±90 ms) — cukup responsif.
  Motor lebih cepat = latensi lebih kecil tapi titik per revolusi berkurang.

## 4. Kalibrasi (urutan yang disarankan)

1. **Stage / Ruang** — isi ukuran ruang nyata (mis. X `8.00` m, Y `4.00` m) dan posisi LiDAR
   (x, y dalam meter). Tombol cepat: `Tengah`, `Sudut kiri atas`, `Bawah tengah`.
   Marker kuning di preview menunjukkan posisi LiDAR; **drag marker** untuk menggeser,
   `Alt+drag` untuk memutar heading, `Shift` untuk snap 0.25 m / 5°.
2. **Kanvas** — pilih aspek (`16:9` default, `Auto` mengikuti ruang, `32:9`, `4:3`, `Custom`)
   dan resolusi (`Auto` tinggi 1080 → lebar mengikuti aspek, atau `Fixed` W×H).
   `Fit` mengatur bagaimana ruang dipetakan: `Stretch` (isi penuh), `Fit` (letterbox), `Fill` (crop).
   Panel menampilkan hasil, mis. `Hasil: 1920×1080  px/m: 240 × 270`; kalau px/m X ≠ Y artinya
   ada distorsi skala (normal kalau aspek ruang ≠ aspek kanvas).
3. **Crop zona** — batasi sudut/jarak scan kalau ada area yang tidak perlu dipakai.
4. **Mapping proyektor** — `Edit quad di preview` lalu drag 4 sudut untuk keystone proyektor
   (quad tersimpan ternormalisasi 0–1, jadi aman saat resolusi berubah).
5. **Belajar background** — tombol `Belajar background` (default otomatis 4 detik saat start).
   Berdiri di luar area, biarkan ruangan kosong.
6. **Simpan config** — tombol `Simpan config` atau `S`, tersimpan di `config.json`.

Kalau terlalu banyak track palsu: naikkan `Toleransi bg (m)` (0.12 → 0.2) dan/atau `Masuk (rev)` 3.

Catatan resolusi: default sekarang **16:9 → 1920×1080**. Untuk 2 proyektor 3840×1080,
pilih aspek `32:9` (auto) atau gunakan `--width 3840 --height 1080`. Perubahan resolusi
diterapkan otomatis saat aplikasi berjalan (renderer, mask, dan kedua Spout sender di-resize).

### Areas (trigger zone)

Area adalah region di ruang nyata (meter) tempat kehadiran orang dihitung. Bisa banyak area,
bentuk **rectangle** atau **polygon**, dan hanya tampil sebagai overlay di preview
(tidak masuk output Spout kecuali `areas.show_in_output = true`).

- **Tambah Rectangle**: tekan `+ Rectangle` di panel, lalu drag di preview.
- **Tambah Polygon**: tekan `+ Polygon`, klik titik sudut satu per satu;
  klik titik awal atau `Enter` untuk menutup; `Backspace` hapus titik terakhir; `Esc` batal.
  Alternatif cepat: `Ctrl+klik` di preview langsung memulai polygon.
- **Pilih & geser**: klik di dalam area → area terpilih (outline lebih tebal) dan bisa digeser
  dengan drag. Panah keyboard menggeser 0.1 m (`Shift` 0.5 m).
- **Edit vertex**: aktifkan `Unlock vertex` (atau tekan `A`), lalu drag handle putih di sudut.
  `Shift` = snap grid 0.25 m.
- **Hapus / duplikat**: `Hapus` atau `Del`; `Duplikat` atau `Ctrl+D`.
- **Properti**: nama (dipakai jadi slug untuk OSC), warna, enable per area, dan `Overlap`
  (track dihitung di semua area yang memuatnya; kalau mati hanya area teratas).
- `Aktifkan layer area` mematikan seluruh layer tanpa menghapus data.
- **Output Spout = area**: pilih area lalu tekan `Spout: hanya area ini`. Kanvas, mask, dan
  koordinat `u,v` akan mengikuti **bounding box area** (di-rescale sesuai `Fit`), sehingga
  yang tampil di proyektor hanya area trigger itu. Tombol `Spout: kembali ke seluruh ruang`
  untuk membatalkan. Berlaku untuk rectangle maupun polygon.
- Semua area tersimpan di `config.json` (`areas.items`) saat `Simpan config` / `S`.
- **Nilai live** tampil di panel dan di label preview: jumlah orang di area `(7)`.
  Data per area juga dikirim via OSC (lihat bagian OSC).

## 5. Output Spout

| Sender | Isi |
|---|---|
| `Lidar Beauty` | titik glow + trail + sweep + grid + ring track, background transparan (RGBA) |
| `Lidar Mask` | blob putih di hitam untuk area yang bergerak (mask/trigger) |

Di Resolume: tambahkan source **Spout** → pilih `Lidar Beauty`. Untuk mask, tambahkan layer
dengan source `Lidar Mask` dan pakai sebagai mask/opacity, atau kirim ke efek.

Catatan: gambar dikirim dengan flip vertikal (`spout.invert = true`) supaya orientasinya benar
di Resolume. Kalau di aplikasi lain tampil terbalik, ubah `spout.invert` di `config.json`.

## 6. OSC

Default: `127.0.0.1:9000`, 30 pesan/detik.

| Address | Nilai |
|---|---|
| `/zone/presence` | `0` / `1` |
| `/zone/count` | jumlah kaki terdeteksi |
| `/zone/enter`, `/zone/exit` | id track yang masuk/keluar |
| `/foot/<id>/x`, `y` | posisi di ruang lantai (meter, koordinat LiDAR) |
| `/foot/<id>/u`, `v` | posisi ternormalisasi kanvas (0–1) — pakai ini untuk Resolume/TD |
| `/foot/<id>/vx`, `vy` | kecepatan (m/s) |
| `/foot/<id>/size` | ukuran cluster (m) |
| `/foot/<id>/active` | `1` |

Per area (kalau area aktif, `slug` dari nama area):

| Address | Nilai |
|---|---|
| `/area/<slug>/presence` | `0` / `1` |
| `/area/<slug>/count` | jumlah orang di area |
| `/area/<slug>/cx`, `/cy` | centroid di ruang (meter) |
| `/area/<slug>/u`, `/v` | centroid ternormalisasi kanvas (0–1) |
| `/area/<slug>/speed` | kecepatan rata-rata (m/s) |
| `/area/<slug>/enter`, `/exit` | id track masuk/keluar area |

### TUIO 2Dcur

Default: `127.0.0.1:3333`, 30 frame/detik. Setiap frame dikirim satu bundle berisi
`/tuio/2Dcur alive`, `set` untuk setiap kaki (`s x y X Y V`), dan `fseq`.

- Koordinat `x, y` = posisi ternormalisasi kanvas (0–1, origin kiri-atas), sama dengan `u, v` di OSC.
- `V` = kecepatan ternormalisasi (meter/detik dibagi ukuran ruang).
- Session ID = ID track (stabil selama orang terlihat).
- `tuio.area_filter` (opsional) diisi slug area → hanya track di area itu yang dikirim.
- TouchDesigner: `TUIO In DAT/CHOP` di port 3333. Resolume: kompatibel lewat TUIO/OSC eksternal.

Contoh di TouchDesigner: `OSC In CHOP` → port 9000, lalu `Select CHOP` per alamat.
Di Resolume: `Preferences → OSC` aktif, lalu map parameter/clip ke alamat di atas.

## 7. Performa

- Target 60 fps di kanvas 3840×1080; di Intel UHD realistis 45–60 fps tergantung beban.
- Kalau fps turun: kecilkan `Ukuran titik`, matikan `Trail`, atau jalankan `--headless`
  (preview memakan waktu komposit).
- Mask dikirim maksimal 30 fps (`spout.mask_fps`) untuk menghemat bandwidth Spout.

## 8. Troubleshooting

| Gejala | Sebab / solusi |
|---|---|
| `Gagal membuka COM4 ... Access denied` | port dipakai app lain (mis. `LIDARLOMBOK_BARU.exe`, YDLidarViewer). Tutup dulu. |
| `LIDAR: TIDAK ADA DATA`, paket 0 | kabel TX/RX terbalik, GND tidak nyambung, atau LiDAR belum dapat 5 V. |
| Paket rusak (`rusak` naik) | baud salah / kabel terlalu panjang / noise motor — pakai kabel pendek, pastikan ground. |
| Gambar terbalik/mirror | ubah `Heading` di Stage, `Mirror Y (scan)`, atau `spout.invert`. |
| Skala terlihat gepeng | aspek kanvas ≠ aspek ruang: ganti `Fit` ke `Fit`, atau aspek ke `Auto`. |
| Banyak track palsu | belajar ulang background saat ruangan kosong, naikkan toleransi bg. |
| Track hilang saat orang diam | turunkan `Keluar (rev)` atau naikkan `Masuk (rev)` → lebih toleran. |
| Titik hanya sedikit | area terlalu jauh (>5 m) atau permukaan gelap; pindahkan LiDAR lebih dekat. |

## 9. Struktur & tool development

```
src/
  app.py                 main loop, mapping, OSC, Spout
  config.py              default + load/save config.json
  lidar/protocol.py      parser YDLIDAR AA55 (checksum XOR, koreksi sudut)
  lidar/reader.py        reader thread (serial & replay) + buffer revolusi
  lidar/geometry.py      polar→kartesian, crop, homography
  lidar/stage.py         stage: ruang + posisi LiDAR + kanvas (aspek, fit, auto resolusi)
  lidar/background.py    model background multi-hipotesis per bin sudut
  lidar/tracker.py       clustering + tracking kaki (ID, velocity, hysteresis)
  ui/areas.py            model area (rect/polygon) + hit-test + persist config
  ui/area_editor.py      state machine editor area (drag, polygon, vertex)
  render/gl.py           primitive GL (titik glow, mesh, ping-pong FBO)
  render/renderer.py     renderer utama (trail, sweep, grid, ring)
  render/mask.py         renderer motion mask
  output/spout_out.py    Spout sender (sendFbo, GPU sharing)
  output/osc_out.py      OSC encoder UDP
  output/tuio_out.py     TUIO 2Dcur sender (bundle alive/set/fseq)
  ui/panel.py            panel imgui
tools/
  record.py              rekam serial ke .bin + statistik
  make_demo.py           buat rekaman demo sintetis (tanpa hardware)
  spout_receiver_test.py cek sender Spout benar-benar mengirim frame
  osc_listen_test.py     dengarkan pesan OSC
  tuio_listen_test.py    dengarkan bundle TUIO 2Dcur
  debug_background.py    analisa background model dari rekaman
  smoke_all.py           uji gabungan Spout + OSC + TUIO dalam satu sesi
```

Cek cepat kesehatan sistem:

```bat
python tools\smoke_all.py                              :: Spout + OSC + TUIO sekaligus (hardware)
python tools\smoke_all.py --replay data\live_rec.bin    :: tanpa hardware
python tools\record.py --seconds 4 --out data\cek.bin
python tools\spout_receiver_test.py --name "Lidar Beauty" --seconds 3
python tools\osc_listen_test.py --seconds 3
python tools\tuio_listen_test.py --seconds 3
```

## 10. Build executable

```bat
build_exe.bat
```

Menghasilkan `dist\LidarToSpout\LidarToSpout.exe` (PyInstaller, mode console supaya log terlihat).
