/* =========================================================
   ARC LIVE LINK
   When the page is served by arc_ui_bridge (ROS 2), the shopping list
   drives the real stack: stops in the simulated store, live behaviour-tree
   mode, robot pose and arrivals. Opened as a plain file, the UI keeps its
   original offline demo behaviour.
========================================================= */
(function () {
  const STATE_LABEL = {
    IDLE: "Idle", NAVIGATING: "On the way", AT_STOP: "At the shelf", PAUSED: "Paused",
    STOPPED: "Stopped", RETURNING: "Returning to start", DONE: "Trip finished"
  };
  const STOP_STATE = { pending: "Up next", active: "Heading here", reached: "Picked", skipped: "Skipped" };
  // map colours (SVG needs real values): aisle colours come from the page's CSS tokens
  const css = (name, fallback) => (getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback);
  const colourOf = (v) => (v && v.startsWith("var(") ? css(v.slice(4, -1), "#8a949d") : v);
  const CHECKOUT_COLOUR = "#2c2f7a";
  const byIdMap = () => new Map(products.map((p) => [p.id, p]));
  function stopAisle(st) {
    const first = (st.products || [])[0];
    const p = first && byIdMap().get(first.id);
    return aisleSign(p ? p.aisle : "");
  }

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
      <div class="nav-info-row"><span>Connection</span><strong id="liveLink">Live, ROS 2</strong></div>
      <div class="nav-info-row"><span>Behaviour tree</span><strong id="liveBt">—</strong></div>
      <div class="nav-info-row"><span>Route</span><strong id="liveOrder">—</strong></div>
      <div class="nav-info-row"><span>Robot position</span><strong id="livePose">—</strong></div>
      <div class="arc-live-actions">
        <button class="primary-button" id="liveNextBtn" type="button">Item picked, next stop</button>
        <button class="secondary-button" id="liveSkipBtn" type="button">Skip this stop</button>
      </div>
      <div class="arc-live-events" id="liveEvents"></div>`;
    aside.insertBefore(card, document.getElementById("routeOrder").nextSibling);
    // robot view under the concept map: the real store the robot drives in
    const mp = document.querySelector(".map-panel");
    const box = el("div", { id: "liveMapBox", style: "margin-top:18px" });
    box.innerHTML = `<div class="panel-heading"><div><h2>Robot in the store</h2>
        <span>Live view of the simulated store. Numbers are the visiting order in aisle colours, C is the checkout, the red arrow is ARC and the square is the start.</span></div></div>
      <svg id="liveMap" class="arc-live-map" viewBox="0 0 300 220" aria-label="Robot in the simulated store"></svg>`;
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
    if (nb) nb.textContent = "Stop (software)";
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
    pts.forEach((p) => { h += `<circle cx="${X(p[0])}" cy="${Y(p[1])}" r="2.5" fill="#c3cbc7"/>`; });
    if (stops.length) {
      const seq = stops.map((t) => `${X(t.x)},${Y(t.y)}`).join(" ");
      h += `<polyline points="${seq}" fill="none" stroke="#17212b" stroke-opacity="0.45" stroke-width="2" stroke-dasharray="4 3"/>`;
    }
    stops.forEach((t, i) => {
      const done = t.status === "reached" || t.status === "skipped";
      const ring = t.status === "active" ? ` stroke="#17212b" stroke-width="2.5"` : ` stroke="#ffffff" stroke-width="2"`;
      const op = done ? ` opacity="0.45"` : "";
      if (t.is_checkout) {
        const cx = X(t.x), cy = Y(t.y);
        h += `<g${op}><polygon points="${cx},${cy - 12} ${cx + 12},${cy} ${cx},${cy + 12} ${cx - 12},${cy}" fill="${CHECKOUT_COLOUR}"${ring}/>` +
             `<text x="${cx}" y="${cy + 4}" text-anchor="middle" font-size="11" fill="#fff" font-weight="700">C</text></g>`;
        return;
      }
      const c = colourOf(stopAisle(t).colour);
      h += `<g${op}><circle cx="${X(t.x)}" cy="${Y(t.y)}" r="9.5" fill="${c}"${ring}/>` +
           `<text x="${X(t.x)}" y="${Y(t.y) + 4}" text-anchor="middle" font-size="10.5" fill="#fff" font-weight="700">${i + 1}</text></g>`;
    });
    if (s.trip && s.trip.home) {
      const hm = s.trip.home;
      h += `<rect x="${X(hm.x) - 6}" y="${Y(hm.y) - 6}" width="12" height="12" fill="#ffffff" stroke="#17212b" stroke-width="2"/>`;
    }
    if (s.pose) {
      const [x, y, th] = s.pose, cx = X(x), cy = Y(y), r = 13;
      const p1 = [cx + r * Math.cos(-th), cy + r * Math.sin(-th)];
      const p2 = [cx + 0.6 * r * Math.cos(-th + 2.5), cy + 0.6 * r * Math.sin(-th + 2.5)];
      const p3 = [cx + 0.6 * r * Math.cos(-th - 2.5), cy + 0.6 * r * Math.sin(-th - 2.5)];
      h += `<polygon points="${p1} ${p2} ${p3}" fill="#c4291c" stroke="#ffffff" stroke-width="1.5"/>`;
    }
    svg.innerHTML = h;
  }

  function render(s) {
    link.last = s;
    const t = s.trip || {};
    const atCheckout = t.state === "DONE" && (t.stops || []).some((x) => x.is_checkout && x.status === "reached");
    const label = atCheckout ? "Finished at checkout"
      : (t.to_checkout && t.state === "NAVIGATING") ? "Heading to checkout" : (STATE_LABEL[t.state] || t.state || "Idle");
    dom.systemStatusText.textContent = label;
    dom.cartMode.textContent = label;
    dom.navMode.textContent = label;
    dom.nextStop.textContent = t.current ? (t.current === "CHECKOUT" ? "Checkout counter" : ((byIdMap().get(t.current) || {}).name || t.current.replaceAll("_", " "))) : "—";
    dom.navStatusBadge.textContent = label;
    const loc = document.getElementById("locValue");
    if (loc) loc.textContent = s.localised ? "Located" : "Not located";
    const bat = document.getElementById("batteryValue");
    if (bat) bat.textContent = "Not in sim";
    document.getElementById("liveBt").textContent =
      s.bt_mode ? `${s.bt_mode}${s.bt_age_s > 3 ? ` (${s.bt_age_s}s old)` : ""}` : "no /arc/behavior_mode";
    const info = t.info || {};
    const routeName = info.route === "typed" ? "as typed" : info.route === "optimised" ? "optimised" : info.strategy;
    document.getElementById("liveOrder").textContent = info.strategy
      ? `${routeName[0].toUpperCase()}${routeName.slice(1)}${info.cost !== undefined ? `, ${info.cost} m` : ""}${info.route !== "typed" && info.cost_as_entered ? ` (typed ${info.cost_as_entered} m)` : ""}${info.ends_at === "checkout" ? ", ends at checkout" : ""}`
      : "—";
    document.getElementById("livePose").textContent = s.pose ? `${s.pose[0].toFixed(2)}, ${s.pose[1].toFixed(2)}` : "—";
    const nextBtn = document.getElementById("liveNextBtn");
    nextBtn.disabled = t.state !== "AT_STOP";
    const upcoming = (t.stops || [])[(t.index || 0) + 1];
    nextBtn.textContent = upcoming && upcoming.is_checkout ? "Item picked, go to checkout" : "Item picked, next stop";

    const rm = document.getElementById("routeMethod");
    if (rm && t.stops && t.stops.length) rm.textContent = "ARC's order drawn on the concept layout (illustrative). Live stops are on the right.";
    const stops = t.stops || [];
    if (stops.length) {
      dom.routeOrder.innerHTML = stops.map((st, i) => {
        const state = st.is_checkout ? ({ pending: "Last stop", active: "Heading here", reached: "Arrived", skipped: "Skipped" }[st.status] || st.status)
          : (STOP_STATE[st.status] || st.status);
        if (st.is_checkout) {
          return `<div class="arc-live-stop is-checkout" data-status="${st.status}">
          <div class="stop-top"><span class="stop-name">${i + 1}. Checkout counter</span>
            <span class="stop-state">${state}</span></div>
          <div class="stop-items">Every trip ends here</div></div>`;
        }
        const sign = stopAisle(st);
        const prods = (st.products || []).map((p) => `${escapeHTML(p.name)}${p.qty > 1 ? ` ×${p.qty}` : ""}`).join(", ");
        const where = (st.products || [])[0] && byIdMap().get(st.products[0].id);
        return `<div class="arc-live-stop" data-status="${st.status}" style="--aisle:${sign.colour}">
          <div class="stop-top"><span class="stop-name">${i + 1}. ${prods || escapeHTML(st.label.replaceAll("_", " "))}</span>
            <span class="stop-state">${state}</span></div>
          <div class="stop-items">${where ? escapeHTML(where.aisle + (where.unit && where.unit !== where.aisle ? `, ${where.unit}` : "")) : escapeHTML(st.label)}</div></div>`;
      }).join("");
    }
    const ev = (t.events || []).slice().reverse();
    document.getElementById("liveEvents").innerHTML = ev.map((e) => `${e.t} s: ${escapeHTML(e.text)}`).join("<br>");
    if (s.other_goal_publishers > 0 && !["NAVIGATING", "RETURNING"].includes(t.state)) {
      document.getElementById("liveLink").textContent = "Close shopping_manager first";
    }
    const ends = document.getElementById("routeEnds");
    if (ends) ends.textContent = s.checkout ? "From the start, through your items, to the checkout" : "From the start, through your items, and back";
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
    renderGivenRoute(ordered, "ARC's order drawn on the concept layout (illustrative)");
    const info = s.trip.info || {};
    const nItems = s.trip.stops.filter((x) => !x.is_checkout).length;
    if (info.cost !== undefined) dom.routeDistance.textContent = `${info.cost} m (store map)`;
    dom.routeStops.textContent = `${nItems} stop${nItems === 1 ? "" : "s"}${s.trip.stops.some((x) => x.is_checkout) ? " + checkout" : ""}`;
  }

  /* Route choice: both routes priced by the bridge on the live map, from the robot. */
  function chooseRoute(plan) {
    return new Promise((resolve) => {
      const o = plan.options, saved = plan.saved_m || 0;
      const fmt = (st) => (st === "CHECKOUT" ? "Checkout counter" : ((byIdMap().get(st) || {}).name || st.replaceAll("_", " ")));
      const sign = (st) => { const p = byIdMap().get(st); return aisleSign(p ? p.aisle : ""); };
      const card = (key, title, opt, tag, cta) => `
        <button type="button" data-route="${key}" class="arc-route-card${key === "optimised" ? " recommended" : ""}">
          <span class="card-head"><span class="card-title">${title}</span>
            <span class="card-metres">${opt.cost.toFixed(1)} m</span></span>
          <span class="card-note">${tag}</span>
          <ol>${opt.stops.map((st) => st === "CHECKOUT" ? `<li class="is-checkout">Checkout counter</li>`
              : `<li style="--aisle:${sign(st).colour}">${escapeHTML(fmt(st))}</li>`).join("")}</ol>
          <span class="card-cta">${cta}</span></button>`;
      const ov = el("div", { id: "arcRouteChooser", class: "arc-chooser-backdrop", role: "dialog", "aria-modal": "true", "aria-label": "Choose a route" });
      ov.innerHTML = `
        <div class="arc-chooser">
          <h2>Choose your route</h2>
          <p class="lede">Both routes are measured through the aisles from where ARC is now${plan.checkout ? " and end at the checkout counter" : ""}.
            ${saved > 0.005 ? `The shortest route saves <strong>${saved.toFixed(1)} m (${Math.round(plan.saved_pct)}%)</strong>.` : "Your order is already the shortest."}</p>
          <div class="arc-route-grid">
            ${card("optimised", "Shortest route", o.optimised, `ARC reorders your list (${o.optimised.strategy === "exact" ? "Held–Karp, exact" : o.optimised.strategy})`, "Use the shortest route")}
            ${card("typed", "My order", o.typed, "Items in the order you added them", "Keep my order")}
          </div>
          ${(o.optimised.unreachable || []).length ? `<p class="warn">Not reachable on the map, so visited last: ${escapeHTML(o.optimised.unreachable.join(", "))}</p>` : ""}
          <div class="chooser-foot">
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
        if (!route) { showToast("Trip not started. Your list is unchanged."); return; }
      } else if (!plan.options) {
        showToast(`ARC: ${plan.reason || "route options unavailable"}`);
      }
      const s = await api("/api/trip", route ? Object.assign({}, req, { route }) : req);
      switchScreen("navigation");
      showConcept(s);
      render(s);
      const nItems = s.trip.stops.filter((x) => !x.is_checkout).length;
      showToast(`Shopping started: ${nItems} stop${nItems === 1 ? "" : "s"}${s.trip.stops.some((x) => x.is_checkout) ? ", then the checkout" : ""}, ${route === "typed" ? "in your order" : route === "optimised" ? "on the shortest route" : s.trip.info.strategy}.`);
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
      const said = { next: "Next stop", skip: "Stop skipped", pause: "Paused", resume: "Resumed", stop: "Stopped", home: "Returning to start" };
      showToast(s.warning ? `ARC: ${s.warning}` : `${said[action] || action}.`);
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
      spans[1].innerHTML = `<strong>${escapeHTML(p.aisle)}</strong>`;
      spans[2].textContent = isAisle ? p.unit : "Stand-alone fixture";
      if (p.spot) card.title = `Stop at ${p.spot} in the store map`;
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
    if (added.length) showToast(`Added ${added.map((id) => (byKey.get(id.toLowerCase()) || {}).name || id).join(", ")}`);
    unknown.forEach((u) => showToast(`"${u}" is not in the store database and will not be added.`));
    return { added, unknown };
  }
  link.addTyped = addTyped;

  function setupStoreMode() {
    const title = document.querySelector("#shopScreen .page-title-row h1");
    const sub = document.querySelector("#shopScreen .page-title-row p");
    if (title) title.textContent = "Shop";
    if (sub) sub.textContent = "Type your list or tap items below. These are the items in the ARC store, the same ones the shopping manager knows.";
    const cat = document.querySelector(".catalog-panel .panel-heading h2");
    if (cat) cat.textContent = "Store items";
    dom.searchInput.placeholder = "Search the store";
    const up = dom.uniqueProducts && dom.uniqueProducts.previousElementSibling;
    if (up) up.textContent = "Different items";
    dom.generateRouteBtn.style.display = "none";          // START SHOPPING shows both routes instead
    dom.startShoppingBtn.textContent = "Start shopping";
    dom.startShoppingBtn.classList.replace("secondary-button", "primary-button");

    const bar = el("form", { id: "arcAddForm", class: "search-wrap", autocomplete: "off" });
    bar.innerHTML = `
      <div class="arc-add-row">
        <input id="arcAddInput" type="text" list="arcItemNames" aria-label="Type items, separated by commas"
          placeholder="Type items, separated by commas: pasta, tea_coffee, milk" style="flex:1" />
        <button class="primary-button" type="submit">Add</button>
      </div>
      <p class="arc-add-hint">Works like the shopping manager prompt. You choose the route when you start.</p>
      <datalist id="arcItemNames">${products.map((p) => `<option value="${escapeHTML(p.id)}">${escapeHTML(p.name)}</option>`).join("")}</datalist>`;
    const row = document.querySelector("#shopScreen .page-title-row");
    row.parentNode.insertBefore(bar, row.nextSibling);
    bar.addEventListener("submit", (e) => {
      e.preventDefault();
      const inp = document.getElementById("arcAddInput");
      if (!inp.value.trim()) { showToast("Type at least one item, for example: milk, pasta"); return; }
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
      document.getElementById("liveLink").textContent = "Bridge not answering";
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
      showToast("Connected to ARC. Your list now drives the robot.");
    } catch (e) {
      /* no bridge: keep the offline demo */
    }
  }

  window.addEventListener("load", detect);
})();
