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
  const CHECKOUT_COLOUR = "#7c3aed";

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
        <span>Simulated store map · numbers = robot visiting order · C = checkout (final stop) · red = robot · square = home</span></div></div>
      <svg id="liveMap" viewBox="0 0 300 220" style="width:100%;max-height:360px;background:#fbfcfe;border:1px solid #e4e8ee;border-radius:8px" aria-label="Robot in the simulated store"></svg>`;
    mp.appendChild(box);
    const sub = document.getElementById("routeMethod");
    if (sub) sub.dataset.live = "1";
    document.querySelectorAll(".route-panel .panel-heading span").forEach((sp) => {
      if (sp.textContent.trim() === "Home → products → Home") { sp.id = "routeEnds"; }
    });
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
    if (s.checkout) all.push(s.checkout);
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
      if (t.is_checkout) {
        const c = t.status === "pending" ? CHECKOUT_COLOUR : (STATUS_COLOUR[t.status] || CHECKOUT_COLOUR);
        const cx = X(t.x), cy = Y(t.y);
        h += `<polygon points="${cx},${cy - 11} ${cx + 11},${cy} ${cx},${cy + 11} ${cx - 11},${cy}" fill="${c}"/>` +
             `<text x="${cx}" y="${cy + 4}" text-anchor="middle" font-size="10" fill="#fff" font-weight="700">C</text>`;
        return;
      }
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
    const atCheckout = t.state === "DONE" && (t.stops || []).some((x) => x.is_checkout && x.status === "reached");
    const label = atCheckout ? "AT CHECKOUT · DONE"
      : (t.to_checkout && t.state === "NAVIGATING") ? "TO CHECKOUT" : (STATE_LABEL[t.state] || t.state || "IDLE");
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
    const routeName = info.route === "typed" ? "as typed" : info.route === "optimised" ? "optimised" : info.strategy;
    document.getElementById("liveOrder").textContent = info.strategy
      ? `${routeName}${info.cost !== undefined ? ` · ${info.cost} m` : ""}${info.route !== "typed" && info.cost_as_entered ? ` (typed ${info.cost_as_entered} m)` : ""}${info.ends_at === "checkout" ? " · ends at checkout" : ""}`
      : "—";
    document.getElementById("livePose").textContent = s.pose ? `${s.pose[0].toFixed(2)}, ${s.pose[1].toFixed(2)}` : "—";
    const nextBtn = document.getElementById("liveNextBtn");
    nextBtn.disabled = t.state !== "AT_STOP";
    nextBtn.style.opacity = nextBtn.disabled ? 0.5 : 1;
    const upcoming = (t.stops || [])[(t.index || 0) + 1];
    nextBtn.textContent = upcoming && upcoming.is_checkout ? "ITEM PICKED · TO CHECKOUT" : "ITEM PICKED · NEXT";

    const rm = document.getElementById("routeMethod");
    if (rm && t.stops && t.stops.length) rm.textContent = "Robot's order on the concept layout (illustrative) · live stops on the right";
    const stops = t.stops || [];
    if (stops.length) {
      dom.routeOrder.innerHTML = stops.map((st, i) => {
        if (st.is_checkout) {
          return `<div class="arc-live-stop" style="display:block;border:1px solid #e4e8ee;border-left:4px solid ${CHECKOUT_COLOUR};border-radius:8px;padding:8px 10px;margin-bottom:8px;background:#faf5ff">
          <div style="display:flex;justify-content:space-between;align-items:center;gap:8px">
            <strong>${i + 1}. Checkout counter</strong>
            <span style="font-size:11px;font-weight:700;color:${st.status === "pending" ? CHECKOUT_COLOUR : STATUS_COLOUR[st.status]}">${st.status === "pending" ? "FINAL STOP" : st.status.toUpperCase()}</span></div>
          <div style="font-size:12px;color:#64748b;margin-top:2px">Every trip ends here</div></div>`;
        }
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
    const ends = document.getElementById("routeEnds");
    if (ends) ends.textContent = s.checkout ? "Start → products → checkout counter" : "Home → products → Home";
    drawMap(s);
  }

  /* The concept layout on the navigation screen shows the robot's order (not its own optimisation).
     Its distances are those of the concept layout, so the header shows the robot's path length. */
  function showConcept(s) {
    const byId = new Map(products.map((p) => [p.id, p]));
    const ordered = [];
    (s.trip.stops || []).forEach((st) => {
      if (st.is_checkout) return;
      (st.products || []).forEach((p) => { const pr = byId.get(p.id); if (pr && !ordered.includes(pr)) ordered.push(pr); });
    });
    renderGivenRoute(ordered, "Robot's order on the concept layout (illustrative)");
    const info = s.trip.info || {};
    const nItems = s.trip.stops.filter((x) => !x.is_checkout).length;
    if (info.cost !== undefined) dom.routeDistance.textContent = `${info.cost} m (store map)`;
    dom.routeStops.textContent = `${nItems} stop${nItems === 1 ? "" : "s"}${s.trip.stops.some((x) => x.is_checkout) ? " + checkout" : ""}`;
  }

  /* Route choice: both routes priced by the bridge on the live map, from the robot. */
  function chooseRoute(plan) {
    return new Promise((resolve) => {
      const o = plan.options, saved = plan.saved_m || 0;
      const card = (key, title, opt, tag) => `
        <button type="button" data-route="${key}" class="arc-route-card" style="text-align:left;border:2px solid ${key === "optimised" ? "var(--blue)" : "var(--border)"};
          background:var(--panel);border-radius:10px;padding:14px;cursor:pointer;display:flex;flex-direction:column;gap:6px;font:inherit;color:inherit">
          <span style="display:flex;justify-content:space-between;gap:8px;align-items:baseline">
            <strong style="font-size:15px">${title}</strong>
            <span style="font-size:20px;font-weight:700">${opt.cost.toFixed(2)} m</span></span>
          <span style="font-size:12px;color:var(--muted)">${tag}</span>
          <ol style="margin:4px 0 0;padding-left:20px;font-size:13px;line-height:1.5">
            ${opt.stops.map((s) => `<li${s === "CHECKOUT" ? ` style="color:${CHECKOUT_COLOUR};font-weight:700"` : ""}>${escapeHTML(s === "CHECKOUT" ? "Checkout counter" : s.replaceAll("_", " "))}</li>`).join("")}
          </ol></button>`;
      const ov = el("div", { id: "arcRouteChooser", role: "dialog", "aria-modal": "true", "aria-label": "Choose a route",
        style: "position:fixed;inset:0;background:rgba(15,23,42,.45);display:flex;align-items:center;justify-content:center;z-index:9999;padding:16px" });
      ov.innerHTML = `
        <div style="background:var(--bg);border-radius:14px;max-width:720px;width:100%;max-height:90vh;overflow:auto;padding:18px;box-shadow:0 20px 50px rgba(0,0,0,.25)">
          <h2 style="margin:0 0 4px;font-size:18px">Choose the route</h2>
          <p style="margin:0 0 12px;color:var(--muted);font-size:13px">Both are Dijkstra path lengths through the aisles, from where the robot is now${plan.checkout ? ", ending at the checkout counter" : ""}.
            ${saved > 0.005 ? `The optimised route is <strong>${saved.toFixed(2)} m (${Math.round(plan.saved_pct)}%) shorter</strong>.` : "Your order is already the shortest."}</p>
          <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px">
            ${card("optimised", "Optimised route", o.optimised, `Shortest order (${o.optimised.strategy === "exact" ? "Held–Karp, exact" : o.optimised.strategy})`)}
            ${card("typed", "My order", o.typed, "The order of your list")}
          </div>
          ${(o.optimised.unreachable || []).length ? `<p style="color:var(--red);font-size:12px;margin:10px 0 0">Not reachable on the map (visited last): ${escapeHTML(o.optimised.unreachable.join(", "))}</p>` : ""}
          <div style="display:flex;justify-content:flex-end;margin-top:12px">
            <button type="button" class="secondary-button" data-route="">Cancel</button></div>
        </div>`;
      const done = (v) => { document.removeEventListener("keydown", onKey); ov.remove(); resolve(v); };
      const onKey = (e) => { if (e.key === "Escape") done(null); };
      ov.addEventListener("click", (e) => {
        const b = e.target.closest("[data-route]");
        if (b) done(b.dataset.route || null);
        else if (e.target === ov) done(null);
      });
      document.addEventListener("keydown", onKey);
      document.body.appendChild(ov);
      ov.querySelector('[data-route="optimised"]').focus();
    });
  }

  link.start = async function () {
    const req = createShoppingRequest();
    if (!req) return;
    try {
      let route;
      const plan = await api("/api/plan", req);
      if (plan.ask && plan.options) {
        route = await chooseRoute(plan);
        if (!route) { showToast("Trip not started."); return; }
      } else if (!plan.options) {
        showToast(`ARC: ${plan.reason || "route options unavailable"}`);
      }
      const s = await api("/api/trip", route ? Object.assign({}, req, { route }) : req);
      switchScreen("navigation");
      showConcept(s);
      render(s);
      const nItems = s.trip.stops.filter((x) => !x.is_checkout).length;
      showToast(`Trip sent to ARC: ${nItems} stops${s.trip.stops.some((x) => x.is_checkout) ? " + checkout" : ""} (${route === "typed" ? "your order" : route === "optimised" ? "optimised" : s.trip.info.strategy}).`);
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

  /* Store catalogue (catalog:=store): the page lists the Gazebo store's own items, and a type-in box
     works like the shopping manager's "Add items (comma separated)" prompt. */
  function storeLabels() {
    const n = products.length;
    dom.productCount.textContent = `${n} store item${n === 1 ? "" : "s"}`;
    const shown = getFilteredProducts().length;
    dom.catalogSubtext.textContent = shown === n ? "All store items" : `Showing ${shown} of ${n} store items`;
    const sel = getSelectedProducts().length;
    dom.selectedMini.textContent = `${sel} selected`;
    // the category is the aisle here, so show it once: "Aisle: Aisle 1" -> "Aisle 1", fixtures as "Area"
    const byId = new Map(products.map((p) => [p.id, p]));
    dom.productGrid.querySelectorAll(".product-card").forEach((card) => {
      const p = byId.get(card.dataset.id), spans = card.querySelectorAll(".product-meta span");
      if (!p || spans.length < 2) return;
      spans[0].remove();
      const isAisle = /^Aisle \d/.test(p.aisle);
      spans[1].innerHTML = `<strong>${isAisle ? "Aisle" : "Area"}:</strong> ${escapeHTML(isAisle ? p.aisle.slice(6) : p.aisle)}`;
    });
  }

  function addTyped(text) {
    const byKey = new Map();
    products.forEach((p) => { byKey.set(p.id.toLowerCase(), p); byKey.set(p.name.toLowerCase(), p); });
    const added = [], unknown = [];
    text.split(",").map((s) => s.trim().toLowerCase()).filter(Boolean).forEach((tok) => {
      const p = byKey.get(tok) || byKey.get(tok.replace(/\s+/g, "_")) || byKey.get(tok.replace(/_/g, " "));
      if (!p) { unknown.push(tok); return; }
      selectedQuantities[p.id] = Number(selectedQuantities[p.id] || 0) + 1;
      added.push(p.id);
    });
    renderProducts();
    renderShoppingList();
    updateSummary();
    clearRouteBecauseListChanged();
    if (added.length) showToast(`Added: ${added.join(", ")}`);
    unknown.forEach((u) => showToast(`"${u}" is not in the store database and will not be added.`));
    return { added, unknown };
  }
  link.addTyped = addTyped;

  function setupStoreMode() {
    const title = document.querySelector("#shopScreen .page-title-row h1");
    const sub = document.querySelector("#shopScreen .page-title-row p");
    if (title) title.textContent = "Shopping List";
    if (sub) sub.textContent = "Items in the ARC store: the same list the shopping manager uses";
    const cat = document.querySelector(".catalog-panel .panel-heading h2");
    if (cat) cat.textContent = "Store items";
    dom.searchInput.placeholder = "Search store items...";
    const up = dom.uniqueProducts && dom.uniqueProducts.previousElementSibling;
    if (up) up.textContent = "Different items";
    dom.generateRouteBtn.style.display = "none";          // START SHOPPING shows both routes instead
    dom.startShoppingBtn.textContent = "START SHOPPING · CHOOSE ROUTE";
    dom.startShoppingBtn.classList.replace("secondary-button", "primary-button");

    const bar = el("form", { id: "arcAddForm", class: "search-wrap", autocomplete: "off",
      style: "display:flex;gap:8px;align-items:stretch" });
    bar.innerHTML = `
      <input id="arcAddInput" type="text" list="arcItemNames" aria-label="Add items, comma separated"
        placeholder="Add items (comma separated), e.g. pasta, tea_coffee, milk" style="flex:1" />
      <button class="primary-button" type="submit" style="width:auto;padding:0 22px">ADD</button>
      <datalist id="arcItemNames">${products.map((p) => `<option value="${escapeHTML(p.id)}">${escapeHTML(p.name)}</option>`).join("")}</datalist>`;
    const row = document.querySelector("#shopScreen .page-title-row");
    row.parentNode.insertBefore(bar, row.nextSibling);
    bar.addEventListener("submit", (e) => {
      e.preventDefault();
      const inp = document.getElementById("arcAddInput");
      if (!inp.value.trim()) { showToast("No items entered. Please try again."); return; }
      const r = addTyped(inp.value);
      inp.value = r.unknown.join(", ");
    });

    const orig = window.renderProducts;
    window.renderProducts = function () { orig.apply(this, arguments); storeLabels(); };
    renderProducts();
  }

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
      if (s.catalog === "store") {
        for (let i = 0; i < 50 && !products.length; i++) await new Promise((r) => setTimeout(r, 100));
        setupStoreMode();
      }
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
