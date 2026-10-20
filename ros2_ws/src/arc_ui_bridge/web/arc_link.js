/* =========================================================
   ARC LIVE LINK
   When the page is served by arc_ui_bridge (ROS 2), the shopping list
   drives the real stack: stops in the simulated store, live behaviour-tree
   mode, robot pose and arrivals. Opened as a plain file, the UI keeps its
   original offline demo behaviour.
========================================================= */
(function () {
  const STATE_LABEL = {
    IDLE: "IDLE", NAVIGATING: "ROUTE ACTIVE", AT_STOP: "AT STOP", PAUSED: "PAUSED",
    STOPPED: "SOFTWARE STOP", RETURNING: "RETURNING HOME", DONE: "TRIP DONE"
  };
  const STATUS_COLOUR = { pending: "#94a3b8", active: "#2563eb", reached: "#16a34a", skipped: "#d97706" };

  const link = { live: false, last: null };
  window.arcLink = link;

  async function api(path, body) {
    const opts = body === undefined ? { cache: "no-store" }
      : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
    const r = await fetch(path, opts);
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
    return data;
  }

  function el(tag, attrs, html) {
    const e = document.createElement(tag);
    Object.entries(attrs || {}).forEach(([k, v]) => e.setAttribute(k, v));
    if (html !== undefined) e.innerHTML = html;
    return e;
  }

  function buildPanel() {
    const aside = document.querySelector(".route-panel");
    const card = el("div", { class: "nav-info-card", id: "liveCard" });
    card.innerHTML = `
      <div class="nav-info-row"><span>Link</span><strong id="liveLink">ROS 2 · live</strong></div>
      <div class="nav-info-row"><span>Behaviour tree</span><strong id="liveBt">—</strong></div>
      <div class="nav-info-row"><span>Order</span><strong id="liveOrder">—</strong></div>
      <div class="nav-info-row"><span>Robot (map)</span><strong id="livePose">—</strong></div>
      <div style="display:flex;gap:8px;margin-top:8px">
        <button class="primary-button" id="liveNextBtn" type="button" style="flex:1">ITEM PICKED · NEXT</button>
        <button class="secondary-button" id="liveSkipBtn" type="button" style="flex:1">SKIP STOP</button>
      </div>
      <div id="liveEvents" style="font-size:12px;color:#64748b;margin-top:8px;max-height:120px;overflow:auto"></div>`;
    aside.insertBefore(card, document.getElementById("routeOrder").nextSibling);
    // robot view under the concept map: the real store the robot drives in
    const mp = document.querySelector(".map-panel");
    const box = el("div", { id: "liveMapBox", style: "margin-top:14px" });
    box.innerHTML = `<div class="panel-heading"><div><h2>Robot in the store (live)</h2>
        <span>Simulated store map · numbers = robot visiting order · red = robot · square = home</span></div></div>
      <svg id="liveMap" viewBox="0 0 300 220" style="width:100%;max-height:360px;background:#fbfcfe;border:1px solid #e4e8ee;border-radius:8px" aria-label="Robot in the simulated store"></svg>`;
    mp.appendChild(box);
    const sub = document.getElementById("routeMethod");
    if (sub) sub.dataset.live = "1";
    document.getElementById("liveNextBtn").addEventListener("click", () => link.control("next"));
    document.getElementById("liveSkipBtn").addEventListener("click", () => link.control("skip"));
    ["estopBtn", "estopTopBtn", "navEStopBtn"].forEach((id) => {
      const b = document.getElementById(id);
      if (b) b.title = "Software stop: holds the robot at its current pose. The hardware E-stop is the safety device.";
    });
    const nb = document.getElementById("navEStopBtn");
    if (nb) nb.textContent = "STOP (SOFTWARE)";
  }

  function drawMap(s) {
    const svg = document.getElementById("liveMap");
    const pts = Object.values(s.store_items || {});
    const stops = (s.trip && s.trip.stops) || [];
    const all = pts.concat(stops.map((x) => [x.x, x.y]));
    if (s.pose) all.push([s.pose[0], s.pose[1]]);
    if (!all.length) { svg.innerHTML = ""; return; }
    const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]);
    const minx = Math.min(...xs) - 0.6, maxx = Math.max(...xs) + 0.6;
    const miny = Math.min(...ys) - 0.6, maxy = Math.max(...ys) + 0.6;
    const sc = Math.min(280 / (maxx - minx), 200 / (maxy - miny));
    const X = (x) => 10 + (x - minx) * sc, Y = (y) => 210 - (y - miny) * sc;   // map y up
    let h = "";
    pts.forEach((p) => { h += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="3" fill="#cbd5e1"/>`; });
    if (stops.length) {
      const seq = stops.map((t) => `${X(t.x)},${Y(t.y)}`).join(" ");
      h += `<polyline points="${seq}" fill="none" stroke="#93c5fd" stroke-width="2" stroke-dasharray="4 3"/>`;
    }
    stops.forEach((t, i) => {
      const c = STATUS_COLOUR[t.status] || "#94a3b8";
      h += `<circle cx="${X(t.x)}" cy="${Y(t.y)}" r="9" fill="${c}"/>` +
           `<text x="${X(t.x)}" y="${Y(t.y) + 4}" text-anchor="middle" font-size="10" fill="#fff" font-weight="700">${i + 1}</text>`;
    });
    if (s.trip && s.trip.home) {
      const hm = s.trip.home;
      h += `<rect x="${X(hm.x) - 6}" y="${Y(hm.y) - 6}" width="12" height="12" fill="none" stroke="#0f172a" stroke-width="2"/>`;
    }
    if (s.pose) {
      const [x, y, th] = s.pose, cx = X(x), cy = Y(y), r = 9;
      const p1 = [cx + r * Math.cos(-th), cy + r * Math.sin(-th)];
      const p2 = [cx + 0.6 * r * Math.cos(-th + 2.5), cy + 0.6 * r * Math.sin(-th + 2.5)];
      const p3 = [cx + 0.6 * r * Math.cos(-th - 2.5), cy + 0.6 * r * Math.sin(-th - 2.5)];
      h += `<polygon points="${p1} ${p2} ${p3}" fill="#dc2626"/>`;
    }
    svg.innerHTML = h;
  }

  function render(s) {
    link.last = s;
    const t = s.trip || {};
    const label = STATE_LABEL[t.state] || t.state || "IDLE";
    dom.systemStatusText.textContent = label;
    dom.cartMode.textContent = s.bt_mode ? `${label} · BT ${s.bt_mode}` : label;
    dom.navMode.textContent = label;
    dom.nextStop.textContent = t.current || "—";
    dom.navStatusBadge.textContent = label;
    const loc = document.getElementById("locValue");
    if (loc) loc.textContent = s.localised ? "LOCALISED" : "NO POSE";
    const bat = document.getElementById("batteryValue");
    if (bat) bat.textContent = "n/a (sim)";
    document.getElementById("liveBt").textContent =
      s.bt_mode ? `${s.bt_mode}${s.bt_age_s > 3 ? ` (${s.bt_age_s}s old)` : ""}` : "no /arc/behavior_mode";
    const info = t.info || {};
    document.getElementById("liveOrder").textContent = info.strategy
      ? `${info.strategy}${info.cost !== undefined ? ` · ${info.cost} m` : ""}${info.cost_as_entered ? ` (typed ${info.cost_as_entered} m)` : ""}`
      : "—";
    document.getElementById("livePose").textContent = s.pose ? `${s.pose[0].toFixed(2)}, ${s.pose[1].toFixed(2)}` : "—";
    const nextBtn = document.getElementById("liveNextBtn");
    nextBtn.disabled = t.state !== "AT_STOP";
    nextBtn.style.opacity = nextBtn.disabled ? 0.5 : 1;

    const rm = document.getElementById("routeMethod");
    if (rm && t.stops && t.stops.length) rm.textContent = "Concept layout preview · the robot follows the order on the right";
    const stops = t.stops || [];
    if (stops.length) {
      dom.routeOrder.innerHTML = stops.map((st, i) => {
        const prods = (st.products || []).map((p) => `${escapeHTML(p.name)}${p.qty > 1 ? ` ×${p.qty}` : ""}`).join(", ");
        return `<div class="arc-live-stop" style="display:block;border:1px solid #e4e8ee;border-left:4px solid ${STATUS_COLOUR[st.status]};border-radius:8px;padding:8px 10px;margin-bottom:8px">
          <div style="display:flex;justify-content:space-between;align-items:center;gap:8px">
            <strong>${i + 1}. ${escapeHTML(st.label.replaceAll("_", " "))}</strong>
            <span style="font-size:11px;font-weight:700;color:${STATUS_COLOUR[st.status]}">${st.status.toUpperCase()}</span></div>
          <div style="font-size:12px;color:#64748b;margin-top:2px">${prods}</div></div>`;
      }).join("");
    }
    const ev = (t.events || []).slice().reverse();
    document.getElementById("liveEvents").innerHTML = ev.map((e) => `${e.t}s · ${escapeHTML(e.text)}`).join("<br>");
    if (s.other_goal_publishers > 0 && !["NAVIGATING", "RETURNING"].includes(t.state)) {
      document.getElementById("liveLink").textContent = "ROS 2 · close shopping_manager!";
    }
    drawMap(s);
  }

  link.start = async function () {
    const req = createShoppingRequest();
    if (!req) return;
    optimizeRoute();                         // concept-map preview, as before
    try {
      const s = await api("/api/trip", req);
      switchScreen("navigation");
      render(s);
      showToast(`Trip sent to ARC: ${s.trip.stops.length} stops (${s.trip.info.strategy}).`);
      const pr = (s.trip.info.problems || []);
      if (pr.length) showToast(`Not mapped: ${pr.join("; ")}`);
    } catch (e) {
      showToast(`ARC: ${e.message}`);
    }
  };

  link.control = async function (action) {
    try {
      const s = await api("/api/control", { action });
      render(s);
      showToast(s.warning ? `ARC: ${s.warning}` : `ARC: ${action.toUpperCase()} sent.`);
    } catch (e) {
      showToast(`ARC: ${e.message}`);
    }
  };

  async function poll() {
    try {
      render(await api("/api/status"));
    } catch (e) {
      document.getElementById("liveLink").textContent = "bridge not answering";
    }
  }

  async function detect() {
    if (location.protocol === "file:") return;          // offline demo
    try {
      const s = await api("/api/status");
      if (!s.ros) return;
      link.live = true;
      buildPanel();
      render(s);
      setInterval(poll, 1000);
      showToast("Connected to ARC (ROS 2). Shopping trips now drive the robot.");
    } catch (e) {
      /* no bridge: keep the offline demo */
    }
  }

  window.addEventListener("load", detect);
})();
