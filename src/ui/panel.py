"""Panel kontrol imgui."""
from __future__ import annotations

import imgui


def _section(title: str) -> None:
    imgui.spacing()
    imgui.text_colored(title, 0.5, 0.85, 1.0, 1.0)
    imgui.separator()


def draw_panel(cfg: dict, status: dict) -> dict:
    """Gambar panel. Mengembalikan aksi yang diminta user."""
    actions = {"save": False, "reconnect": False, "reset": False, "quit": False, "edit_quad": False}

    imgui.set_next_window_size(min(460, imgui.get_io().display_size[0] - 20),
                               max(200, imgui.get_io().display_size[1] - 20), imgui.FIRST_USE_EVER)
    imgui.set_next_window_position(10, 10, imgui.FIRST_USE_EVER)
    expanded, opened = imgui.begin("Lidar To Spout", True)
    if not opened:
        imgui.end()
        return actions

    # --- status ---
    _section("Status")
    conn = status.get("connected", False)
    if conn:
        imgui.text_colored("LIDAR: TERHUBUNG", 0.4, 1.0, 0.4, 1.0)
    else:
        imgui.text_colored("LIDAR: TIDAK ADA DATA", 1.0, 0.45, 0.3, 1.0)
    imgui.text(f"FPS: {status.get('fps', 0):.1f}   rotasi: {status.get('rate_hz', 0):.2f} Hz   motor: {status.get('motor_hz', 0):.2f} Hz")
    imgui.text(f"Titik/revolusi: {status.get('points_per_rev', 0)}   total titik: {status.get('points_total', 0)}")
    imgui.text(f"Paket OK: {status.get('packets_ok', 0)}   rusak: {status.get('packets_bad', 0)}")
    if status.get("error"):
        imgui.text_colored(status["error"][:90], 1.0, 0.4, 0.3, 1.0)
    if status.get("spout_beauty"):
        imgui.text_colored("Spout beauty: " + status["spout_beauty"], 0.6, 0.9, 0.6, 1.0)
    if status.get("spout_mask"):
        imgui.text_colored("Spout mask: " + status["spout_mask"], 0.6, 0.9, 0.6, 1.0)

    # --- serial ---
    _section("Sumber data")
    s = cfg["serial"]
    ch, val = imgui.input_text("Port", s.get("port", "COM4"), 32)
    if ch:
        s["port"] = val
    ch, val = imgui.input_int("Baud", int(s.get("baud", 115200)))
    if ch:
        s["baud"] = max(9600, val)
    if imgui.button("Reconnect"):
        actions["reconnect"] = True
    imgui.same_line()
    imgui.text_disabled(f"replay: {s.get('replay_file') or '-'}")

    # --- crop ---
    _section("Crop zona")
    c = cfg["crop"]
    ch, val = imgui.checkbox("Batasi sudut", bool(c.get("use_angle", True)))
    if ch:
        c["use_angle"] = val
    ch, val = imgui.slider_float("Sudut min", float(c.get("angle_min", 0.0)), 0.0, 360.0, "%.1f")
    if ch:
        c["angle_min"] = val
    ch, val = imgui.slider_float("Sudut max", float(c.get("angle_max", 180.0)), 0.0, 360.0, "%.1f")
    if ch:
        c["angle_max"] = val
    ch, val = imgui.slider_float("Jarak min (m)", float(c.get("dist_min", 0.3)), 0.0, 8.0, "%.2f")
    if ch:
        c["dist_min"] = val
    ch, val = imgui.slider_float("Jarak max (m)", float(c.get("dist_max", 5.5)), 0.2, 12.0, "%.2f")
    if ch:
        c["dist_max"] = val

    # --- areas ---
    _section("Areas (trigger zone)")
    store = status.get("area_store")
    editor = status.get("area_editor")
    if store is None or editor is None:
        imgui.text_disabled("Area tidak tersedia")
    else:
        ch, val = imgui.checkbox("Aktifkan layer area", store.enabled)
        if ch:
            store.set_enabled(val)
        if imgui.button("+ Rectangle"):
            editor.arm_rect()
        imgui.same_line()
        if imgui.button("+ Polygon"):
            editor.arm_poly()
        imgui.same_line()
        ch, val = imgui.checkbox("Unlock vertex", editor.unlocked)
        if ch:
            editor.unlocked = val
        if editor.busy:
            imgui.text_colored(f"Mode: {editor.mode}", 1.0, 0.8, 0.3, 1.0)
            imgui.same_line()
            if imgui.button("Batal"):
                editor.cancel()
        metrics = status.get("area_metrics", {})
        for area in list(store.items):
            m = metrics.get(area.id, {})
            count = int(m.get("count", 0))
            label = f"{area.id}. {area.name}"
            if count:
                label += f"   ({count})"
            if imgui.selectable(label, area.id == store.selected_id):
                store.selected_id = area.id
            imgui.same_line()
            ch, val = imgui.checkbox(f"##on{area.id}", area.enabled)
            if ch:
                area.enabled = val
                store.save()
            imgui.same_line()
            imgui.text_disabled(area.slug)
        sel = store.selected
        out_id = int(cfg["areas"].get("output_area", 0) or 0)
        out_area = store.get(out_id)
        imgui.text_disabled(
            f"Output Spout: {'seluruh ruang' if out_area is None else 'area ' + out_area.slug}")
        if sel is not None:
            ch, val = imgui.input_text("Nama area", sel.name, 48)
            if ch:
                sel.name = val
                store.save()
            ch, col = imgui.color_edit3("Warna area", *sel.color)
            if ch:
                sel.color = list(col)
                store.save()
            ch, val = imgui.checkbox("Overlap (bisa masuk beberapa area)", sel.overlap)
            if ch:
                sel.overlap = val
                store.save()
            lm = status.get("area_metrics", {}).get(sel.id, {})
            imgui.text_disabled(
                f"live: presence {int(lm.get('presence', 0))} · count {int(lm.get('count', 0))} · "
                f"u {float(lm.get('u', 0.0)):.3f} v {float(lm.get('v', 0.0)):.3f} · "
                f"speed {float(lm.get('speed', 0.0)):.2f} m/s")
            imgui.text_disabled(f"OSC: /area/{sel.slug}/presence|count|cx|cy|u|v|speed")
            if imgui.button("Duplikat"):
                store.duplicate(sel.id)
            imgui.same_line()
            if imgui.button("Hapus"):
                store.remove(sel.id)
            if out_id == sel.id:
                if imgui.button("Spout: kembali ke seluruh ruang"):
                    cfg["areas"]["output_area"] = 0
            else:
                if imgui.button("Spout: hanya area ini"):
                    cfg["areas"]["output_area"] = int(sel.id)
        else:
            imgui.text_disabled("Belum ada area. Tambah rectangle/polygon.")
        imgui.text_disabled("Ctrl+klik preview = polygon · A = unlock vertex · Del = hapus")

    # --- stage / ruang ---
    _section("Stage / Ruang")
    st = cfg["stage"]
    ch, val = imgui.checkbox("Mode stage (ruang + posisi LiDAR)", bool(st.get("enabled", True)))
    if ch:
        st["enabled"] = val
    room = st["room"]
    ch, val = imgui.drag_float("Ruang X (m)", float(room.get("width_m", 8.0)), 0.05, 0.5, 60.0, "%.2f")
    if ch:
        room["width_m"] = max(0.5, val)
    ch, val = imgui.drag_float("Ruang Y (m)", float(room.get("height_m", 4.0)), 0.05, 0.5, 60.0, "%.2f")
    if ch:
        room["height_m"] = max(0.5, val)

    lidar = st["lidar"]
    ch, val = imgui.drag_float("LiDAR x (m)", float(lidar.get("x_m", 4.0)), 0.02, 0.25, 60.0, "%.2f")
    if ch:
        lidar["x_m"] = val
    ch, val = imgui.drag_float("LiDAR y (m)", float(lidar.get("y_m", 2.0)), 0.02, 0.25, 60.0, "%.2f")
    if ch:
        lidar["y_m"] = val
    ch, val = imgui.drag_float("Heading (deg)", float(lidar.get("heading_deg", 0.0)), 0.2, 1.0, 360.0, "%.1f")
    if ch:
        lidar["heading_deg"] = val
    ch, val = imgui.checkbox("Mirror Y (scan)", bool(lidar.get("mirror_y", False)))
    if ch:
        lidar["mirror_y"] = val

    if imgui.button("Tengah"):
        lidar["x_m"] = float(room["width_m"]) * 0.5
        lidar["y_m"] = float(room["height_m"]) * 0.5
    imgui.same_line()
    if imgui.button("Sudut kiri atas"):
        lidar["x_m"] = 0.0
        lidar["y_m"] = 0.0
    imgui.same_line()
    if imgui.button("Bawah tengah"):
        lidar["x_m"] = float(room["width_m"]) * 0.5
        lidar["y_m"] = float(room["height_m"])
    imgui.text_disabled("Drag marker di preview · Alt+drag = putar heading · Shift = snap")

    # --- kanvas ---
    _section("Kanvas")
    cv = st["canvas"]
    res = st["resolution"]
    aspect_names = ["16:9", "auto", "32:9", "4:3", "custom"]
    aspect_labels = ["16:9 (default)", "Auto (dari ruang)", "32:9", "4:3", "Custom"]
    cur_aspect = str(cv.get("aspect", "16:9"))
    idx = aspect_names.index(cur_aspect) if cur_aspect in aspect_names else 0
    ch, val = imgui.combo("Aspek", idx, aspect_labels)
    if ch:
        cv["aspect"] = aspect_names[val]
    if cv.get("aspect") == "custom":
        ch, val = imgui.drag_float("Aspek custom", float(cv.get("aspect_custom", 2.0)), 0.01, 0.1, 10.0, "%.2f")
        if ch:
            cv["aspect_custom"] = max(0.1, val)

    fits = ["stretch", "fit", "fill"]
    fit_labels = ["Stretch (isi penuh)", "Fit (letterbox)", "Fill (crop)"]
    cur_fit = str(cv.get("fit", "stretch"))
    fidx = fits.index(cur_fit) if cur_fit in fits else 0
    ch, val = imgui.combo("Fit", fidx, fit_labels)
    if ch:
        cv["fit"] = fits[val]

    modes = ["auto_height", "fixed"]
    mode_labels = ["Auto (tinggi → lebar)", "Fixed (W×H)"]
    cur_mode = str(res.get("mode", "auto_height"))
    midx = modes.index(cur_mode) if cur_mode in modes else 0
    ch, val = imgui.combo("Resolusi", midx, mode_labels)
    if ch:
        res["mode"] = modes[val]
    if res.get("mode") == "fixed":
        ch, val = imgui.drag_int("Lebar (px)", int(cfg["canvas"].get("width", 1920)), 1.0, 64, 7680)
        if ch:
            cfg["canvas"]["width"] = val
        ch, val = imgui.drag_int("Tinggi (px)", int(cfg["canvas"].get("height", 1080)), 1.0, 64, 7680)
        if ch:
            cfg["canvas"]["height"] = val
    else:
        ch, val = imgui.drag_int("Tinggi (px)", int(res.get("height", 1080)), 1.0, 64, 4320)
        if ch:
            res["height"] = val
    sx, sy = status.get("px_per_m", (0.0, 0.0))
    imgui.text(f"Hasil: {status.get('canvas_w', 0)}×{status.get('canvas_h', 0)}   "
               f"px/m: {sx:.0f} × {sy:.0f}")
    if abs(sx - sy) > max(1.0, 0.02 * max(sx, sy)):
        imgui.text_colored("Aspek ruang ≠ kanvas (skala tidak seragam)", 1.0, 0.75, 0.3, 1.0)

    # --- mapping proyektor ---
    _section("Mapping proyektor")
    if imgui.button("Reset quad ke full canvas"):
        cfg["map"]["quad"] = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        actions["reset_quad"] = True
    _, val = imgui.checkbox("Edit quad di preview", bool(status.get("edit_quad", False)))
    if _:
        actions["edit_quad"] = bool(val)

    if imgui.collapsing_header("Manual (world_rect) — advanced")[0]:
        m = cfg["map"]
        ch, val = imgui.slider_float("Rotasi peta", float(m.get("angle_offset_deg", 0.0)), -180.0, 180.0, "%.1f")
        if ch:
            m["angle_offset_deg"] = val
        ch, val = imgui.checkbox("Mirror Y (manual)", bool(m.get("mirror_y", False)))
        if ch:
            m["mirror_y"] = val
        ch, vals = imgui.drag_float4("World rect (m)", *[float(v) for v in m["world_rect"]], change_speed=0.05)
        if ch:
            m["world_rect"] = list(vals)
        imgui.text_disabled("Dipakai hanya jika mode stage dimatikan")

    # --- style ---
    _section("Visual")
    st = cfg["style"]
    ch, col = imgui.color_edit3("Warna titik", *st["point_color"])
    if ch:
        st["point_color"] = list(col)
    ch, val = imgui.slider_float("Ukuran titik", float(st.get("point_size", 5.0)), 1.0, 30.0, "%.1f")
    if ch:
        st["point_size"] = val
    ch, val = imgui.slider_float("Alpha titik", float(st.get("point_alpha", 0.9)), 0.0, 1.0, "%.2f")
    if ch:
        st["point_alpha"] = val
    ch, val = imgui.checkbox("Trail", bool(st.get("trail_enabled", True)))
    if ch:
        st["trail_enabled"] = val
    ch, val = imgui.slider_float("Decay trail", float(st.get("trail_decay", 0.87)), 0.0, 0.99, "%.3f")
    if ch:
        st["trail_decay"] = val
    ch, val = imgui.slider_float("Intensitas trail", float(st.get("trail_intensity", 1.0)), 0.0, 3.0, "%.2f")
    if ch:
        st["trail_intensity"] = val
    ch, val = imgui.checkbox("Sweep fan", bool(st.get("sweep_enabled", True)))
    if ch:
        st["sweep_enabled"] = val
    ch, val = imgui.slider_float("Lebar sweep", float(st.get("sweep_width_deg", 55.0)), 5.0, 180.0, "%.1f")
    if ch:
        st["sweep_width_deg"] = val
    ch, val = imgui.slider_float("Alpha sweep", float(st.get("sweep_alpha", 0.1)), 0.0, 0.6, "%.3f")
    if ch:
        st["sweep_alpha"] = val
    ch, col = imgui.color_edit3("Warna sweep", *st["sweep_color"])
    if ch:
        st["sweep_color"] = list(col)
    ch, val = imgui.checkbox("Grid", bool(st.get("grid_enabled", True)))
    if ch:
        st["grid_enabled"] = val
    ch, val = imgui.slider_float("Grid step (m)", float(st.get("grid_step_m", 1.0)), 0.25, 5.0, "%.2f")
    if ch:
        st["grid_step_m"] = val
    ch, val = imgui.slider_float("Alpha grid", float(st.get("grid_alpha", 0.3)), 0.0, 1.0, "%.2f")
    if ch:
        st["grid_alpha"] = val
    ch, val = imgui.checkbox("Ring track", bool(st.get("track_ring", True)))
    if ch:
        st["track_ring"] = val
    ch, val = imgui.slider_float("Radius ring (px)", float(st.get("track_ring_size", 44.0)), 4.0, 200.0, "%.0f")
    if ch:
        st["track_ring_size"] = val

    # --- tracker ---
    _section("Deteksi kaki")
    flt = cfg["filter"]
    ch, val = imgui.checkbox("Filter background (buang objek statis)", bool(flt.get("enabled", True)))
    if ch:
        flt["enabled"] = val
    ch, val = imgui.checkbox("Tampilkan titik mentah", bool(flt.get("show_all", False)))
    if ch:
        flt["show_all"] = val
    ch, val = imgui.slider_float("Toleransi bg (m)", float(flt.get("tol_m", 0.12)), 0.03, 0.50, "%.2f")
    if ch:
        flt["tol_m"] = val
    if imgui.button("Belajar background"):
        actions["learn"] = True
    imgui.same_line()
    imgui.text_disabled(f"{status.get('bg_ratio', 0) * 100:.0f}% terpetakan")

    t = cfg["tracker"]
    ch, val = imgui.checkbox("Aktif", bool(t.get("enabled", True)))
    if ch:
        t["enabled"] = val
    ch, val = imgui.slider_float("Gap cluster (m)", float(t.get("cluster_gap_m", 0.18)), 0.05, 0.6, "%.2f")
    if ch:
        t["cluster_gap_m"] = val
    ch, val = imgui.slider_int("Min titik", int(t.get("min_points", 2)), 1, 10)
    if ch:
        t["min_points"] = val
    ch, val = imgui.slider_float("Gate track (m)", float(t.get("gate_m", 0.55)), 0.1, 2.0, "%.2f")
    if ch:
        t["gate_m"] = val
    ch, val = imgui.slider_int("Masuk (rev)", int(t.get("enter_revs", 2)), 1, 10)
    if ch:
        t["enter_revs"] = val
    ch, val = imgui.slider_int("Keluar (rev)", int(t.get("exit_revs", 4)), 1, 20)
    if ch:
        t["exit_revs"] = val

    # --- mask ---
    _section("Mask / trigger")
    mk = cfg["mask"]
    ch, val = imgui.checkbox("Kirim mask Spout", bool(mk.get("enabled", True)))
    if ch:
        mk["enabled"] = val
    ch, val = imgui.combo("Sumber", 0 if mk.get("source", "tracks") == "tracks" else 1, ["tracks", "points"])
    if ch:
        mk["source"] = "tracks" if val == 0 else "points"
    ch, val = imgui.slider_float("Skala blob", float(mk.get("blob_scale", 3.0)), 0.5, 10.0, "%.1f")
    if ch:
        mk["blob_scale"] = val
    ch, val = imgui.slider_float("Intensitas", float(mk.get("intensity", 1.0)), 0.0, 2.0, "%.2f")
    if ch:
        mk["intensity"] = val
    ch, val = imgui.slider_float("Decay mask", float(mk.get("decay", 0.0)), 0.0, 0.98, "%.2f")
    if ch:
        mk["decay"] = val
    ch, val = imgui.checkbox("Invert", bool(mk.get("invert", False)))
    if ch:
        mk["invert"] = val

    # --- output ---
    _section("Output")
    sp = cfg["spout"]
    imgui.text(f"Spout: {sp.get('beauty')}  |  {sp.get('mask')}")
    o = cfg["osc"]
    ch, val = imgui.checkbox("OSC", bool(o.get("enabled", True)))
    if ch:
        o["enabled"] = val
    ch, val = imgui.input_text("Host", o.get("host", "127.0.0.1"), 32)
    if ch:
        o["host"] = val
    ch, val = imgui.input_int("Port", int(o.get("port", 9000)))
    if ch:
        o["port"] = val

    t = cfg["tuio"]
    ch, val = imgui.checkbox("TUIO 2Dcur", bool(t.get("enabled", True)))
    if ch:
        t["enabled"] = val
    ch, val = imgui.input_text("TUIO host", t.get("host", "127.0.0.1"), 32)
    if ch:
        t["host"] = val
    ch, val = imgui.input_int("TUIO port", int(t.get("port", 3333)))
    if ch:
        t["port"] = val
    ch, val = imgui.drag_float("TUIO rate (Hz)", float(t.get("rate_hz", 30.0)), 0.5, 1.0, 120.0, "%.0f")
    if ch:
        t["rate_hz"] = val
    ch, val = imgui.input_text("Filter area (slug)", t.get("area_filter", ""), 48)
    if ch:
        t["area_filter"] = val
    st = status.get("tuio_status")
    if st:
        imgui.text_disabled(st)

    # --- actions ---
    _section("Aksi")
    if imgui.button("Simpan config"):
        actions["save"] = True
    imgui.same_line()
    if imgui.button("Reset trail"):
        actions["reset"] = True
    imgui.same_line()
    if imgui.button("Quit"):
        actions["quit"] = True
    imgui.text_disabled("Hotkey: H panel, P preview, Q quad, S save, Esc quit")

    imgui.end()
    return actions
