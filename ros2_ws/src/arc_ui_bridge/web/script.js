let products = [];
let selectedQuantities = {};
let activeCategory = "All";
let searchTerm = "";
let optimizedRoute = [];
let routeTotalDistance = 0;
let systemState = "READY";

/*
=========================================================
SUPERMARKET NAVIGATION MODEL

The aisle centre is the walkable aisle.
Shelf edges are drawn on either side of it.
The cart can move:
  1. vertically inside an aisle
  2. horizontally through the top cross-aisle
  3. horizontally through the bottom cross-aisle

Therefore the displayed route never cuts through a shelf.
The route optimizer uses shortest-path distances on this
walkable aisle network, then solves the shopping-stop order.
=========================================================
*/

const HOME = {
  id: "HOME",
  name: "HOME",
  x: 0,
  y: 0,
  aisle: "Entry"
};

let AISLES = [];
let BOTTOM_Y = 1;
let TOP_Y = 15;

const $ = (id) => document.getElementById(id);

const dom = {
  shopScreen: $("shopScreen"),
  navigationScreen: $("navigationScreen"),
  shopTabBtn: $("shopTabBtn"),
  navTabBtn: $("navTabBtn"),
  estopTopBtn: $("estopTopBtn"),
  estopBtn: $("estopBtn"),
  navEStopBtn: $("navEStopBtn"),
  searchInput: $("searchInput"),
  categoryBar: $("categoryBar"),
  productGrid: $("productGrid"),
  productCount: $("productCount"),
  catalogSubtext: $("catalogSubtext"),
  selectedMini: $("selectedMini"),
  shoppingList: $("shoppingList"),
  shoppingListMeta: $("shoppingListMeta"),
  clearListBtn: $("clearListBtn"),
  totalQuantity: $("totalQuantity"),
  uniqueProducts: $("uniqueProducts"),
  generateRouteBtn: $("generateRouteBtn"),
  startShoppingBtn: $("startShoppingBtn"),
  cartMode: $("cartMode"),
  systemStatusText: $("systemStatusText"),
  routeSvg: $("routeSvg"),
  routeLine: $("routeLine"),
  routePoints: $("routePoints"),
  homePoint: $("homePoint"),
  aisleLines: $("aisleLines"),
  mapEmptyText: $("mapEmptyText"),
  routeOrder: $("routeOrder"),
  routeMethod: $("routeMethod"),
  routeDistance: $("routeDistance"),
  routeStops: $("routeStops"),
  navStatusBadge: $("navStatusBadge"),
  navMode: $("navMode"),
  nextStop: $("nextStop"),
  navItemCount: $("navItemCount"),
  resumeBtn: $("resumeBtn"),
  pauseBtn: $("pauseBtn"),
  returnHomeBtn: $("returnHomeBtn"),
  toast: $("toast")
};

function normalizeProduct(product) {
  return {
    id: product.id,
    name: product.name ?? product.ui_name ?? "Unnamed product",
    category: product.category ?? "Other",
    aisle: product.aisle ?? "—",
    zone: product.zone ?? "—",
    x: Number(product.x) || 0,
    y: Number(product.y) || 0,
    yaw: Number(product.yaw) || 0,
    unit: product.unit || "Standard pack",
    icon: product.icon || "🛒",
    source_url: product.source_url || ""
  };
}

async function loadProducts() {
  try {
    const response = await fetch("products.json", { cache: "no-store" });
    if (!response.ok) throw new Error(`products.json returned ${response.status}`);

    const data = await response.json();
    products = Array.isArray(data) ? data.map(normalizeProduct) : [];

    buildAisleModel();
    createCategories();
    drawAisleBackground();
    renderProducts();
    renderShoppingList();
    updateSummary();
    renderEmptyRoute();

    showToast(`${products.length} products loaded.`);
  } catch (error) {
    console.error(error);
    dom.productGrid.innerHTML = `
      <div class="empty-state" style="grid-column: 1 / -1;">
        Could not load products.json.<br />
        Open this project through Live Server or another local web server.
      </div>
    `;
    dom.productCount.textContent = "0 products";
  }
}

/* =========================================================
   AISLE MODEL
========================================================= */

function buildAisleModel() {
  const map = new Map();

  products.forEach((product) => {
    if (!map.has(product.aisle)) map.set(product.aisle, []);
    map.get(product.aisle).push(product);
  });

  AISLES = [...map.entries()]
    .map(([name, list]) => ({
      name,
      x: list.reduce((sum, p) => sum + p.x, 0) / list.length,
      minY: Math.min(...list.map((p) => p.y)),
      maxY: Math.max(...list.map((p) => p.y))
    }))
    .sort((a, b) => a.x - b.x);

  const minProductY = Math.min(...products.map((p) => p.y));
  const maxProductY = Math.max(...products.map((p) => p.y));

  BOTTOM_Y = minProductY - 1;
  TOP_Y = maxProductY + 1;
}

function aisleForProduct(product) {
  return AISLES.find((a) => a.name === product.aisle);
}

/* =========================================================
   CATEGORIES
========================================================= */

function createCategories() {
  const categories = ["All", ...new Set(products.map((p) => p.category))];

  dom.categoryBar.innerHTML = categories
    .map(
      (category) => `
        <button
          type="button"
          class="category-chip ${category === activeCategory ? "active" : ""}"
          data-category="${escapeHTML(category)}"
        >
          ${escapeHTML(category)}
        </button>
      `
    )
    .join("");

  dom.categoryBar.querySelectorAll(".category-chip").forEach((button) => {
    button.addEventListener("click", () => {
      activeCategory = button.dataset.category;
      createCategories();
      renderProducts();
    });
  });
}

function getFilteredProducts() {
  const needle = searchTerm.trim().toLowerCase();

  return products.filter((product) => {
    const matchesCategory =
      activeCategory === "All" || product.category === activeCategory;

    const matchesSearch =
      !needle ||
      product.name.toLowerCase().includes(needle) ||
      product.category.toLowerCase().includes(needle) ||
      product.aisle.toLowerCase().includes(needle) ||
      product.zone.toLowerCase().includes(needle);

    return matchesCategory && matchesSearch;
  });
}

/* =========================================================
   PRODUCT CATALOG
========================================================= */

function renderProducts() {
  const list = getFilteredProducts();
  const selectedCount = Object.keys(selectedQuantities).filter(
    (id) => selectedQuantities[id] > 0
  ).length;

  dom.productCount.textContent = `${products.length} products`;
  dom.catalogSubtext.textContent =
    activeCategory === "All"
      ? `Showing ${list.length} products`
      : `${activeCategory} • ${list.length} products`;
  dom.selectedMini.textContent = `${selectedCount} selected`;

  if (!list.length) {
    dom.productGrid.innerHTML = `
      <div class="empty-state" style="grid-column: 1 / -1;">
        No products match your search.
      </div>
    `;
    return;
  }

  dom.productGrid.innerHTML = list.map(productCardHTML).join("");

  dom.productGrid.querySelectorAll(".product-card").forEach((card) => {
    card.addEventListener("click", () => toggleProduct(card.dataset.id));
  });

  dom.productGrid.querySelectorAll("[data-action='toggle']").forEach((button) => {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      toggleProduct(button.dataset.id);
    });
  });

  dom.productGrid.querySelectorAll("[data-action='inc']").forEach((button) => {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      increaseQuantity(button.dataset.id);
    });
  });

  dom.productGrid.querySelectorAll("[data-action='dec']").forEach((button) => {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      decreaseQuantity(button.dataset.id);
    });
  });
}

function productCardHTML(product) {
  const quantity = Number(selectedQuantities[product.id] || 0);
  const selected = quantity > 0;
  const unit = product.unit || "Standard pack";

  return `
    <article
      class="product-card ${selected ? "selected" : ""}"
      data-id="${escapeHTML(product.id)}"
      tabindex="0"
    >
      <div class="product-icon" aria-hidden="true">
        ${escapeHTML(product.icon)}
      </div>

      <div class="product-name">
        ${escapeHTML(product.name)}
      </div>

      <div class="product-meta">
        <span>${escapeHTML(product.category)}</span>
        <span><strong>Aisle:</strong> ${escapeHTML(product.aisle)}</span>
        <span><strong>${escapeHTML(unit)}</strong></span>
      </div>

      <div class="card-bottom">
        ${
          selected
            ? `
              <div class="main-qty">
                <button
                  class="qty-button"
                  type="button"
                  data-action="dec"
                  data-id="${escapeHTML(product.id)}"
                >−</button>
                <div class="qty-value">QTY ${quantity}</div>
                <button
                  class="qty-button"
                  type="button"
                  data-action="inc"
                  data-id="${escapeHTML(product.id)}"
                >+</button>
              </div>
            `
            : `
              <button
                class="card-add-button"
                type="button"
                data-action="toggle"
                data-id="${escapeHTML(product.id)}"
              >ADD</button>
            `
        }
      </div>
    </article>
  `;
}

function toggleProduct(productId) {
  const current = Number(selectedQuantities[productId] || 0);

  if (current > 0) {
    delete selectedQuantities[productId];
  } else {
    selectedQuantities[productId] = 1;
  }

  renderProducts();
  renderShoppingList();
  updateSummary();
  clearRouteBecauseListChanged();
}

function increaseQuantity(productId) {
  selectedQuantities[productId] =
    Number(selectedQuantities[productId] || 0) + 1;

  renderProducts();
  renderShoppingList();
  updateSummary();
  clearRouteBecauseListChanged();
}

function decreaseQuantity(productId) {
  const next = Number(selectedQuantities[productId] || 0) - 1;

  if (next <= 0) {
    delete selectedQuantities[productId];
  } else {
    selectedQuantities[productId] = next;
  }

  renderProducts();
  renderShoppingList();
  updateSummary();
  clearRouteBecauseListChanged();
}

/* =========================================================
   SHOPPING LIST
========================================================= */

function renderShoppingList() {
  const selected = getSelectedProducts();

  if (!selected.length) {
    dom.shoppingList.innerHTML = `
      <div class="empty-shopping">
        Your shopping list is empty.
      </div>
    `;
    dom.shoppingListMeta.textContent = "0 items";
    return;
  }

  dom.shoppingList.innerHTML = selected
    .map((product) => {
      const qty = Number(selectedQuantities[product.id]);

      return `
        <div class="shopping-item">
          <div class="shopping-item-top">
            <div>
              <div class="shopping-item-name">
                ${escapeHTML(product.name)}
              </div>
              <div class="shopping-item-meta">
                ${escapeHTML(product.aisle)} · ${escapeHTML(product.unit)}
              </div>
            </div>
          </div>

          <div class="shopping-item-controls">
            <button
              type="button"
              data-list-action="dec"
              data-id="${escapeHTML(product.id)}"
            >−</button>
            <div class="shopping-qty">${qty}</div>
            <button
              type="button"
              data-list-action="inc"
              data-id="${escapeHTML(product.id)}"
            >+</button>
          </div>
        </div>
      `;
    })
    .join("");

  dom.shoppingList
    .querySelectorAll("[data-list-action='inc']")
    .forEach((button) => {
      button.addEventListener("click", () => increaseQuantity(button.dataset.id));
    });

  dom.shoppingList
    .querySelectorAll("[data-list-action='dec']")
    .forEach((button) => {
      button.addEventListener("click", () => decreaseQuantity(button.dataset.id));
    });

  dom.shoppingListMeta.textContent =
    `${selected.length} item${selected.length === 1 ? "" : "s"}`;
}

function getSelectedProducts() {
  return products.filter(
    (product) => Number(selectedQuantities[product.id] || 0) > 0
  );
}

function updateSummary() {
  const selected = getSelectedProducts();

  const total = selected.reduce(
    (sum, product) =>
      sum + Number(selectedQuantities[product.id] || 0),
    0
  );

  dom.totalQuantity.textContent = total;
  dom.uniqueProducts.textContent = selected.length;
  dom.selectedMini.textContent = `${selected.length} selected`;
  dom.navItemCount.textContent = selected.length;
}

function clearList() {
  selectedQuantities = {};
  optimizedRoute = [];
  routeTotalDistance = 0;

  renderProducts();
  renderShoppingList();
  updateSummary();
  renderEmptyRoute();

  showToast("Shopping list cleared.");
}

function clearRouteBecauseListChanged() {
  optimizedRoute = [];
  routeTotalDistance = 0;
  renderEmptyRoute();
}

/* =========================================================
   SHORTEST-PATH ROUTING
========================================================= */

function routeDistanceBetween(a, b) {
  if (a.id === "HOME" && b.id === "HOME") return 0;

  if (a.id === "HOME") {
    return homeToProductDistance(b);
  }

  if (b.id === "HOME") {
    return homeToProductDistance(a);
  }

  const aisleA = aisleForProduct(a);
  const aisleB = aisleForProduct(b);

  if (!aisleA || !aisleB) {
    return euclideanDistance(a, b);
  }

  /* Same aisle: stay inside the aisle. */
  if (aisleA.name === aisleB.name) {
    return Math.abs(a.y - b.y);
  }

  /* Different aisles: choose the shorter of top or bottom cross-aisle. */
  const bottom =
    Math.abs(a.y - BOTTOM_Y) +
    Math.abs(b.y - BOTTOM_Y) +
    Math.abs(aisleA.x - aisleB.x);

  const top =
    Math.abs(TOP_Y - a.y) +
    Math.abs(TOP_Y - b.y) +
    Math.abs(aisleA.x - aisleB.x);

  return Math.min(bottom, top);
}

function homeToProductDistance(product) {
  const aisle = aisleForProduct(product);

  if (!aisle) return euclideanDistance(HOME, product);

  /* Home enters the supermarket through the bottom cross-aisle. */
  return (
    Math.abs(HOME.x - aisle.x) +
    Math.abs(HOME.y - BOTTOM_Y) +
    Math.abs(product.y - BOTTOM_Y)
  );
}

function shortestWalkPath(a, b) {
  if (a.id === b.id) return [a];

  if (a.id === "HOME") {
    return homeToProductPath(b);
  }

  if (b.id === "HOME") {
    return reversePath(homeToProductPath(a));
  }

  const aisleA = aisleForProduct(a);
  const aisleB = aisleForProduct(b);

  if (!aisleA || !aisleB) {
    return [a, b];
  }

  /* Same aisle: direct vertical movement. */
  if (aisleA.name === aisleB.name) {
    return [a, b];
  }

  const bottomDistance =
    Math.abs(a.y - BOTTOM_Y) +
    Math.abs(b.y - BOTTOM_Y) +
    Math.abs(aisleA.x - aisleB.x);

  const topDistance =
    Math.abs(TOP_Y - a.y) +
    Math.abs(TOP_Y - b.y) +
    Math.abs(aisleA.x - aisleB.x);

  if (bottomDistance <= topDistance) {
    return [
      a,
      { id: `B-${aisleA.name}`, x: aisleA.x, y: BOTTOM_Y, type: "corridor" },
      { id: `B-${aisleB.name}`, x: aisleB.x, y: BOTTOM_Y, type: "corridor" },
      b
    ];
  }

  return [
    a,
    { id: `T-${aisleA.name}`, x: aisleA.x, y: TOP_Y, type: "corridor" },
    { id: `T-${aisleB.name}`, x: aisleB.x, y: TOP_Y, type: "corridor" },
    b
  ];
}

function homeToProductPath(product) {
  const aisle = aisleForProduct(product);

  if (!aisle) return [HOME, product];

  return [
    HOME,
    { id: `B-${aisle.name}`, x: aisle.x, y: BOTTOM_Y, type: "corridor" },
    product
  ];
}

function reversePath(path) {
  return [...path].reverse();
}

function optimizeRoute() {
  const selected = getSelectedProducts();

  if (!selected.length) {
    renderEmptyRoute();
    showToast("Add at least one product before optimizing the path.");
    return;
  }

  let ordered;
  let method;

  /*
  Exact TSP ordering using shortest aisle-network distances.
  8 stops = 40,320 permutations, which is practical in the browser.
  */
  if (selected.length <= 8) {
    ordered = bruteForceBestRoute(selected);
    method = "Shortest-path optimized route";
  } else {
    ordered = nearestNeighbourRoute(selected);
    method = "Shortest-path route + nearest-neighbour ordering";
  }

  const productSequence = [HOME, ...ordered, HOME];
  const walkPoints = [];

  for (let i = 1; i < productSequence.length; i++) {
    const leg = shortestWalkPath(productSequence[i - 1], productSequence[i]);

    if (i === 1) {
      walkPoints.push(...leg);
    } else {
      walkPoints.push(...leg.slice(1));
    }
  }

  optimizedRoute = walkPoints;
  routeTotalDistance = calculatePolylineDistance(walkPoints);

  renderRoute(method, ordered);

  showToast(
    `Shortest route generated: ${routeTotalDistance.toFixed(2)} m.`
  );
}

function bruteForceBestRoute(items) {
  let bestOrder = items.slice();
  let bestDistance = Infinity;

  function permute(array, start) {
    if (start === array.length) {
      let distance = routeDistanceBetween(HOME, array[0]);

      for (let i = 1; i < array.length; i++) {
        distance += routeDistanceBetween(array[i - 1], array[i]);
      }

      distance += routeDistanceBetween(array[array.length - 1], HOME);

      if (distance < bestDistance) {
        bestDistance = distance;
        bestOrder = array.slice();
      }
      return;
    }

    for (let i = start; i < array.length; i++) {
      [array[start], array[i]] = [array[i], array[start]];
      permute(array, start + 1);
      [array[start], array[i]] = [array[i], array[start]];
    }
  }

  permute(items.slice(), 0);
  return bestOrder;
}

function nearestNeighbourRoute(items) {
  const remaining = items.slice();
  const result = [];
  let current = HOME;

  while (remaining.length) {
    let bestIndex = 0;
    let bestDistance = Infinity;

    remaining.forEach((item, index) => {
      const distance = routeDistanceBetween(current, item);
      if (distance < bestDistance) {
        bestDistance = distance;
        bestIndex = index;
      }
    });

    current = remaining.splice(bestIndex, 1)[0];
    result.push(current);
  }

  return result;
}

function calculatePolylineDistance(route) {
  let total = 0;

  for (let i = 1; i < route.length; i++) {
    total += euclideanDistance(route[i - 1], route[i]);
  }

  return total;
}

function euclideanDistance(a, b) {
  return Math.hypot(
    (b.x ?? 0) - (a.x ?? 0),
    (b.y ?? 0) - (a.y ?? 0)
  );
}

/* =========================================================
   MAP DRAWING
========================================================= */

function drawAisleBackground() {
  /*
  Draw each aisle as two shelf faces with a clear walkable
  centre corridor. The blue route runs down that corridor.
  */

  dom.aisleLines.innerHTML = AISLES
    .map((aisle) => {
      const top = mapPoint(aisle.x, TOP_Y - 0.2);
      const bottom = mapPoint(aisle.x, BOTTOM_Y + 0.2);

      const shelfOffset = 8;

      return `
        <line
          class="shelf-line"
          x1="${top.x - shelfOffset}"
          y1="${top.y}"
          x2="${bottom.x - shelfOffset}"
          y2="${bottom.y}"
        ></line>

        <line
          class="shelf-line"
          x1="${top.x + shelfOffset}"
          y1="${top.y}"
          x2="${bottom.x + shelfOffset}"
          y2="${bottom.y}"
        ></line>

        <text
          class="aisle-label"
          x="${top.x}"
          y="${(top.y + bottom.y) / 2}"
          text-anchor="middle"
        >
          ${escapeHTML(aisle.name)}
        </text>
      `;
    })
    .join("");
}

function renderRoute(method, orderedProducts) {
  if (!optimizedRoute.length) {
    renderEmptyRoute();
    return;
  }

  const points = optimizedRoute.map((point) => mapPoint(point));
  const pointString = points.map((p) => `${p.x},${p.y}`).join(" ");

  dom.routeLine.setAttribute("points", pointString);
  dom.routeLine.setAttribute("opacity", "1");
  dom.mapEmptyText.classList.add("hidden");

  /* Remove duplicate visual markers at corridor turning points. */
  const uniqueProductStops = orderedProducts;

  dom.routePoints.innerHTML = uniqueProductStops
    .map((product, index) => {
      const p = mapPoint(product);

      return `
        <g>
          <circle
            cx="${p.x}"
            cy="${p.y}"
            r="15"
            fill="#2563eb"
            stroke="#ffffff"
            stroke-width="3"
          ></circle>

          <text
            class="route-point-number"
            x="${p.x}"
            y="${p.y + 4}"
          >
            ${index + 1}
          </text>

          <text
            class="route-point-label"
            x="${p.x + 22}"
            y="${p.y - 7}"
          >
            ${escapeHTML(product.name)}
          </text>

          <text
            x="${p.x + 22}"
            y="${p.y + 10}"
            fill="#64748b"
            font-size="11"
          >
            ${escapeHTML(product.aisle)}
          </text>
        </g>
      `;
    })
    .join("");

  const home = mapPoint(HOME);

  dom.homePoint.innerHTML = `
    <g>
      <circle
        cx="${home.x}"
        cy="${home.y}"
        r="17"
        fill="#111827"
        stroke="#ffffff"
        stroke-width="4"
      ></circle>

      <text
        class="home-label"
        x="${home.x + 24}"
        y="${home.y + 5}"
      >HOME</text>
    </g>
  `;

  renderRouteOrder(orderedProducts);

  dom.routeMethod.textContent = method;
  dom.routeDistance.textContent = `${routeTotalDistance.toFixed(2)} m`;
  dom.routeStops.textContent =
    `${orderedProducts.length} stop${orderedProducts.length === 1 ? "" : "s"}`;

  dom.navStatusBadge.textContent = "ROUTE READY";
  dom.navMode.textContent = systemState === "E-STOP" ? "E-STOP" : "READY";
  dom.nextStop.textContent = orderedProducts[0]?.name || "—";
}

function renderRouteOrder(orderedProducts) {
  const sequence = [HOME, ...orderedProducts, HOME];

  dom.routeOrder.innerHTML = sequence
    .map((stop, index) => {
      if (stop.id === "HOME") {
        return `
          <div class="route-stop">
            <div class="route-index">${index === 0 ? "H" : "↩"}</div>
            <div>
              <div class="route-stop-name">HOME</div>
              <div class="route-stop-aisle">
                ${index === 0 ? "Starting point" : "Return point"}
              </div>
            </div>
            <div class="route-stop-dist">
              ${index === 0 ? "START" : "END"}
            </div>
          </div>
        `;
      }

      const previous = sequence[index - 1];
      const leg = routeDistanceBetween(previous, stop);
      const qty = Number(selectedQuantities[stop.id] || 0);

      return `
        <div class="route-stop">
          <div class="route-index">${index}</div>
          <div>
            <div class="route-stop-name">
              ${escapeHTML(stop.name)}
            </div>
            <div class="route-stop-aisle">
              ${escapeHTML(stop.aisle)} · Qty ${qty}
            </div>
          </div>
          <div class="route-stop-dist">
            ${leg.toFixed(2)} m
          </div>
        </div>
      `;
    })
    .join("");
}

function renderEmptyRoute() {
  dom.routeLine.setAttribute("points", "");
  dom.routeLine.setAttribute("opacity", "0");
  dom.routePoints.innerHTML = "";
  dom.homePoint.innerHTML = "";

  dom.routeOrder.innerHTML = `
    <div class="empty-route">
      No route available.<br />
      Add products and press “OPTIMIZE PATH”.
    </div>
  `;

  dom.mapEmptyText.classList.remove("hidden");
  dom.routeMethod.textContent = "Waiting for shopping list";
  dom.routeDistance.textContent = "0.00 m";
  dom.routeStops.textContent = "0 stops";
  dom.navStatusBadge.textContent = "ROUTE NOT GENERATED";
  dom.navMode.textContent = systemState === "E-STOP" ? "E-STOP" : "IDLE";
  dom.nextStop.textContent = "—";
}

function mapPoint(pointOrX, yValue) {
  const point =
    typeof pointOrX === "object"
      ? pointOrX
      : { x: pointOrX, y: yValue };

  const bounds = getMapBounds();

  const left = 70;
  const right = 830;
  const top = 45;
  const bottom = 475;

  const rangeX = Math.max(bounds.maxX - bounds.minX, 1);
  const rangeY = Math.max(bounds.maxY - bounds.minY, 1);

  return {
    x:
      left +
      ((point.x - bounds.minX) / rangeX) * (right - left),
    y:
      bottom -
      ((point.y - bounds.minY) / rangeY) * (bottom - top)
  };
}

function getMapBounds() {
  const xs = [
    HOME.x,
    ...products.map((p) => p.x),
    ...AISLES.map((a) => a.x)
  ];

  const ys = [
    HOME.y,
    BOTTOM_Y,
    TOP_Y,
    ...products.map((p) => p.y)
  ];

  return {
    minX: Math.min(...xs) - 1,
    maxX: Math.max(...xs) + 1,
    minY: Math.min(...ys) - 0.8,
    maxY: Math.max(...ys) + 0.8
  };
}

/* =========================================================
   ROS-READY REQUEST
========================================================= */

function createShoppingRequest() {
  const selected = getSelectedProducts();

  if (!selected.length) {
    showToast("Shopping list is empty.");
    return null;
  }

  const request = {
    items: selected.map((product) => ({
      item_id: product.id,
      name: product.name,
      category: product.category,
      aisle: product.aisle,
      zone: product.zone,
      quantity: Number(selectedQuantities[product.id]),
      x: product.x,
      y: product.y,
      yaw: product.yaw
    }))
  };

  console.log("ROS-ready shopping request:", request);
  return request;
}

/* =========================================================
   CART STATE
========================================================= */

function setSystemState(state) {
  systemState = state;

  const labels = {
    READY: "SYSTEM READY",
    NAVIGATING: "ROUTE ACTIVE",
    PAUSED: "PAUSED",
    "E-STOP": "EMERGENCY STOP",
    "RETURNING HOME": "RETURNING HOME"
  };

  dom.systemStatusText.textContent = labels[state] || state;
  dom.cartMode.textContent = state;
  dom.navMode.textContent = state;

  if (state === "E-STOP") {
    dom.navStatusBadge.textContent = "STOPPED";
  } else if (optimizedRoute.length) {
    dom.navStatusBadge.textContent =
      state === "NAVIGATING" ? "ROUTE ACTIVE" : "ROUTE READY";
  }
}

function startShopping() {
  if (!getSelectedProducts().length) {
    showToast("Add products before starting shopping.");
    return;
  }

  createShoppingRequest();
  optimizeRoute();
  switchScreen("navigation");
  setSystemState("NAVIGATING");
  showToast("ARC navigation started.");
}

function pauseCart() {
  if (systemState === "E-STOP") {
    showToast("E-STOP is active. Press RESUME to reset the cart state.");
    return;
  }

  setSystemState("PAUSED");
  showToast("Cart paused.");
}

function resumeCart() {
  if (!optimizedRoute.length) {
    showToast("Generate a route first.");
    return;
  }

  /*
  This explicitly clears the E-STOP state. The old code only
  changed the mode, leaving the STOPPED badge untouched.
  */
  setSystemState("NAVIGATING");
  dom.navStatusBadge.textContent = "ROUTE ACTIVE";
  dom.navMode.textContent = "NAVIGATING";
  dom.nextStop.textContent = getSelectedProducts()[0]?.name || "—";

  showToast("E-STOP reset. Cart resumed.");
}

function returnHome() {
  if (systemState === "E-STOP") {
    showToast("Reset E-STOP before returning home.");
    return;
  }

  setSystemState("RETURNING HOME");
  dom.nextStop.textContent = "HOME";
  showToast("Return-home command issued.");
}

function emergencyStop() {
  systemState = "E-STOP";
  dom.cartMode.textContent = "EMERGENCY STOP";
  dom.navMode.textContent = "E-STOP";
  dom.systemStatusText.textContent = "EMERGENCY STOP";
  dom.navStatusBadge.textContent = "STOPPED";

  showToast("EMERGENCY STOP ACTIVE.");
}

/* =========================================================
   SCREEN SWITCHING
========================================================= */

function switchScreen(screenName) {
  const showShop = screenName === "shop";

  dom.shopScreen.classList.toggle("active-screen", showShop);
  dom.navigationScreen.classList.toggle("active-screen", !showShop);
  dom.shopTabBtn.classList.toggle("active", showShop);
  dom.navTabBtn.classList.toggle("active", !showShop);

  window.scrollTo({ top: 0, behavior: "smooth" });
}

/* =========================================================
   SEARCH / TOAST / ESCAPE
========================================================= */

dom.searchInput.addEventListener("input", (event) => {
  searchTerm = event.target.value;
  renderProducts();
});

dom.clearListBtn.addEventListener("click", clearList);

dom.generateRouteBtn.addEventListener("click", () => {
  optimizeRoute();
  switchScreen("navigation");
});

dom.startShoppingBtn.addEventListener("click", () => (window.arcLink && arcLink.live ? arcLink.start() : startShopping()));

dom.shopTabBtn.addEventListener("click", () => switchScreen("shop"));
dom.navTabBtn.addEventListener("click", () => switchScreen("navigation"));

const arcStop = () => (window.arcLink && arcLink.live ? arcLink.control("stop") : emergencyStop());
dom.estopBtn.addEventListener("click", arcStop);
dom.estopTopBtn.addEventListener("click", arcStop);
dom.navEStopBtn.addEventListener("click", arcStop);

dom.resumeBtn.addEventListener("click", () => (window.arcLink && arcLink.live ? arcLink.control("resume") : resumeCart()));
dom.pauseBtn.addEventListener("click", () => (window.arcLink && arcLink.live ? arcLink.control("pause") : pauseCart()));
dom.returnHomeBtn.addEventListener("click", () => (window.arcLink && arcLink.live ? arcLink.control("home") : returnHome()));

dom.productGrid.addEventListener("keydown", (event) => {
  if (event.key !== "Enter" && event.key !== " ") return;

  const card = event.target.closest(".product-card");
  if (!card) return;

  event.preventDefault();
  toggleProduct(card.dataset.id);
});

function showToast(message) {
  dom.toast.textContent = message;
  dom.toast.classList.remove("hidden");

  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => {
    dom.toast.classList.add("hidden");
  }, 2400);
}

function escapeHTML(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

loadProducts();
