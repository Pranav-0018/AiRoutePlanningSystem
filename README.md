# Route Algorithm Learning Lab

An interactive web application for learning and visualizing pathfinding algorithms used in GPS navigation and autonomous vehicle routing.

---

## 📁 File Structure

```
route-algo-lab/
├── index.html   — Main HTML markup (structure & layout)
├── styles.css   — All styling, theming, and responsive rules
├── main.js      — All JavaScript (algorithms, rendering, interactions)
└── README.md    — This file
```

---

## 🚀 Getting Started

Open `index.html` in any modern browser. No build step, server, or dependencies required — it runs entirely in the browser.

> **Note:** All three files (`index.html`, `styles.css`, `main.js`) must be in the **same folder** for the app to work correctly.

---

## 🗂️ Pages & Features

### 📚 Page 1 — Learn Algorithms
Click any algorithm card to see a full breakdown: how it works step by step, time/space complexity, and whether it is optimal or complete.

**Algorithms covered:**
| Algorithm   | Type                  | Optimal | Key Trait                          |
|-------------|-----------------------|---------|-------------------------------------|
| A* Search   | Heuristic + Cost      | ✓       | Fastest for point-to-point routing |
| Dijkstra's  | Uniform Cost Search   | ✓       | Exhaustive, guaranteed shortest path|
| BFS         | Breadth-First Search  | Partial | Simple, optimal only on equal costs |
| D* Lite     | Dynamic Replanning    | ✓       | Searches backward, efficient replan|

---

### 🏙️ Page 2 — City Simulator
Run algorithms on real city graphs (Chennai, London, Tokyo, New York) or build your own custom graph.

**Features:**
- **Preset cities** with real-world-inspired node positions and edge distances
- **Manual graph mode**: click canvas to add nodes, connect them with custom distances, drag to reposition
- **Obstacle mode**: block nodes to simulate road closures; algorithms reroute automatically
- **Algorithm comparison**: run all 4 algorithms at once and see a side-by-side table (distance, nodes visited, time)
- **Trace log**: live step-by-step log of every algorithm decision
- **Save/Load scenarios**: 3 local save slots + JSON export/import

---

### 🔬 Page 3 — Step-by-Step Trace
Watch the algorithm make each individual decision, one click at a time.

**Features:**
- Prev / Next navigation through every algorithm event
- Highlighted current node on canvas
- Live display of the internal algorithm state (queue, g-values, visited set)

---

### 🚗 Page 4 — Grid Simulator (AV Route Simulator)
A 26×26 city grid where you place obstacles and watch algorithms navigate around them in real-time animation.

**Features:**
- Draw obstacles by clicking/dragging on road cells
- Set custom start and end positions
- Random obstacle generator
- Animated path visualization with a moving car sprite
- Animated traffic lights at intersections
- Compare all 4 algorithms on the same grid at once

---

## 🎨 Theming

All colors are defined as CSS custom properties in `styles.css` under `:root`. Key variables:

```css
--bg          /* Page background */
--text        /* Primary text */
--astar       /* A* color — blue */
--dijkstra    /* Dijkstra color — amber */
--bfs         /* BFS color — purple */
--dstar       /* D* Lite color — green */
```

To change the color scheme, edit the `:root` block at the top of `styles.css`.

---

## 🔧 Key JavaScript Components (`main.js`)

| Section              | Description                                        |
|----------------------|----------------------------------------------------|
| `ALGO_META`          | Algorithm names, colors, steps, complexity info    |
| `CITIES`             | Preset city graphs (nodes + edges)                 |
| `MinHeap`            | Priority queue used by A*, Dijkstra, D* Lite       |
| `runAstar()`         | A* implementation with trace output                |
| `runDijkstra()`      | Dijkstra's implementation with trace output        |
| `runBfs()`           | BFS implementation with trace output               |
| `runDstar()`         | D* Lite (backward A*) with trace output            |
| `drawCity()`         | Canvas renderer for city graph                     |
| `gridDraw()`         | Canvas renderer for AV grid                        |
| `buildSteps()`       | Generates step-by-step trace for stepper page      |
| `saveToSlot()` etc.  | LocalStorage-based scenario save/load system       |

---

## 🌐 External Resources

- **Fonts** (loaded from Google Fonts):  
  - `JetBrains Mono` — monospace, used for code/data display  
  - `Syne` — UI font for labels and buttons

No other external libraries or frameworks are used.

---

## 🌍 Browser Compatibility

Works in all modern browsers (Chrome, Firefox, Safari, Edge). Requires Canvas 2D API support.
