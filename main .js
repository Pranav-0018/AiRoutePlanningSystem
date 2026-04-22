// ═══════════════════════════════════════════════════════
    // ALGORITHM DEFINITIONS
    // ═══════════════════════════════════════════════════════
    const ALGO_META = {
      astar: {
        name: 'A* Search',
        tag: 'Heuristic + Cost',
        color: '#3a8ff0',
        badge: 'FASTEST · OPTIMAL',
        desc: 'Uses both the actual cost to reach a node and an estimated cost to the goal. Balances exploration with focus, making it the go-to for GPS navigation.',
        complexity: { time: 'O(E log V)', space: 'O(V)' },
        optimal: true, complete: true,
        steps: [
          { t: 'Initialize', d: 'Put the <code>start</code> node into the Open List (priority queue) with priority <code>f = g + h</code>, where g=0 and h = heuristic to goal.' },
          { t: 'Loop — Pick Best Node', d: 'Pop the node with the lowest <code>f</code> value from the Open List. This is the current node <code>u</code>.' },
          { t: 'Goal Check', d: 'If <code>u == goal</code>, we are done! Reconstruct the path by following parent pointers back to start.' },
          { t: 'Expand Neighbors', d: 'For each neighbor <code>v</code> of <code>u</code>: compute <code>g_new = g[u] + edge(u,v)</code>. If this is better than the known g[v], update it.' },
          { t: 'Update Open List', d: 'If <code>v</code> is not in Closed List, push it with priority <code>f = g_new + h(v, goal)</code>.' },
          { t: 'Mark Closed', d: 'Move <code>u</code> to the Closed List so it\'s never processed again.' },
          { t: 'Repeat', d: 'Go back to step 2. If the Open List empties with no path found, no route exists.' }
        ]
      },
      dijkstra: {
        name: "Dijkstra's",
        tag: 'Uniform Cost Search',
        color: '#f0a030',
        badge: 'OPTIMAL · EXHAUSTIVE',
        desc: 'Explores all reachable nodes outward by increasing cost. Guarantees the shortest path but visits more nodes than A* since it has no goal-direction.',
        complexity: { time: 'O((V+E) log V)', space: 'O(V)' },
        optimal: true, complete: true,
        steps: [
          { t: 'Initialize', d: 'Set <code>dist[start] = 0</code> and <code>dist[v] = ∞</code> for all other nodes. Push start into a min-heap.' },
          { t: 'Pop Minimum', d: 'Extract node <code>u</code> with the smallest known distance from the heap.' },
          { t: 'Skip if Processed', d: 'If <code>u</code> is already in the Closed Set, skip it (stale entry).' },
          { t: 'Goal Check', d: 'If <code>u == goal</code>, reconstruct and return the path.' },
          { t: 'Relax Edges', d: 'For each neighbor <code>v</code>: if <code>dist[u] + w(u,v) < dist[v]</code>, update <code>dist[v]</code> and push <code>v</code> into the heap.' },
          { t: 'Repeat', d: 'Continue popping until the heap is empty or goal is found. All min-distances will be correct.' }
        ]
      },
      bfs: {
        name: 'BFS',
        tag: 'Breadth-First Search',
        color: '#a060f0',
        badge: 'OPTIMAL ON EQUAL COST',
        desc: 'Explores level by level — all nodes 1 hop away, then 2 hops, etc. Optimal only when all edges have equal weight, but extremely simple to implement.',
        complexity: { time: 'O(V + E)', space: 'O(V)' },
        optimal: false, complete: true,
        steps: [
          { t: 'Initialize', d: 'Enqueue <code>start</code> into a FIFO queue. Mark it as visited.' },
          { t: 'Dequeue Front', d: 'Pop the first node <code>u</code> from the queue. This is the current node.' },
          { t: 'Goal Check', d: 'If <code>u == goal</code>, trace back through parents and return the path.' },
          { t: 'Enqueue Neighbors', d: 'For each unvisited neighbor <code>v</code> of <code>u</code>: mark <code>v</code> as visited, set <code>parent[v] = u</code>, and enqueue <code>v</code>.' },
          { t: 'Repeat', d: 'Continue until the queue is empty. All nodes at distance <code>d</code> are processed before distance <code>d+1</code>.' }
        ]
      },
      dstar: {
        name: 'D* Lite',
        tag: 'Dynamic Replanning',
        color: '#20d080',
        badge: 'DYNAMIC · BACKWARD',
        desc: 'Searches BACKWARD from goal to start. The key advantage: when the environment changes (new obstacles), only the affected portion of the path is replanned — not the whole thing.',
        complexity: { time: 'O(E log V) per plan', space: 'O(V)' },
        optimal: true, complete: true,
        steps: [
          { t: 'Initialize from Goal', d: 'Put the <code>goal</code> node into the priority queue. All g-values = ∞, rhs-values = ∞, except <code>rhs[goal] = 0</code>.' },
          { t: 'Compute Shortest Path', d: 'Pop node <code>u</code> with minimum key. Update inconsistent nodes — ones where <code>g[u] ≠ rhs[u]</code>.' },
          { t: 'Locally Consistent', d: 'If <code>g[u] > rhs[u]</code> (overconsistent): set <code>g[u] = rhs[u]</code>. Update neighbors.' },
          { t: 'Locally Inconsistent', d: 'If <code>g[u] < rhs[u]</code> (underconsistent): set <code>g[u] = ∞</code>. Put back in queue. Update neighbors.' },
          { t: 'Extract Path', d: 'From start, greedily follow the neighbor with the lowest <code>g + edge_cost</code> value all the way to goal.' },
          { t: 'Dynamic Replanning', d: 'When an obstacle appears: update edge costs, recalculate keys for affected nodes, push them into the queue, and rerun from step 2 — only the changed portion is recomputed.' }
        ]
      }
    };

    // ═══════════════════════════════════════════════════════
    // CITY GRAPHS
    // ═══════════════════════════════════════════════════════
    const CITIES = {
      chennai: {
        nodes: {
          'Chennai Central': { x: 0.42, y: 0.28 },
          'Egmore': { x: 0.40, y: 0.22 },
          'T.Nagar': { x: 0.32, y: 0.45 },
          'Adyar': { x: 0.38, y: 0.65 },
          'Anna Nagar': { x: 0.22, y: 0.30 },
          'Velachery': { x: 0.50, y: 0.72 },
          'Tambaram': { x: 0.44, y: 0.88 },
          'Ambattur': { x: 0.22, y: 0.18 },
          'Porur': { x: 0.18, y: 0.52 },
          'Perambur': { x: 0.48, y: 0.14 },
          'Sholinganallur': { x: 0.64, y: 0.72 },
          'OMR': { x: 0.72, y: 0.55 },
          'Guindy': { x: 0.36, y: 0.58 },
          'Mylapore': { x: 0.45, y: 0.52 },
          'Besant Nagar': { x: 0.46, y: 0.78 },
        },
        edges: [
          ['Chennai Central', 'Egmore', 3],
          ['Chennai Central', 'Perambur', 6],
          ['Chennai Central', 'Mylapore', 8],
          ['Egmore', 'Anna Nagar', 7],
          ['Egmore', 'Ambattur', 12],
          ['Anna Nagar', 'Ambattur', 6],
          ['Anna Nagar', 'Porur', 14],
          ['Anna Nagar', 'T.Nagar', 8],
          ['T.Nagar', 'Guindy', 5],
          ['T.Nagar', 'Adyar', 9],
          ['T.Nagar', 'Mylapore', 6],
          ['Adyar', 'Besant Nagar', 4],
          ['Adyar', 'Velachery', 7],
          ['Guindy', 'Velachery', 6],
          ['Guindy', 'Porur', 10],
          ['Velachery', 'Tambaram', 10],
          ['Velachery', 'Sholinganallur', 8],
          ['Sholinganallur', 'OMR', 7],
          ['Besant Nagar', 'Mylapore', 5],
          ['Mylapore', 'OMR', 12],
          ['Perambur', 'Ambattur', 9],
        ],
        startNode: 'Chennai Central',
        endNode: 'Tambaram'
      },
      london: {
        nodes: {
          'Kings Cross': { x: 0.48, y: 0.18 },
          'Euston': { x: 0.42, y: 0.20 },
          'Paddington': { x: 0.22, y: 0.32 },
          'Victoria': { x: 0.40, y: 0.55 },
          'London Bridge': { x: 0.62, y: 0.50 },
          'Canary Wharf': { x: 0.78, y: 0.45 },
          'Waterloo': { x: 0.48, y: 0.52 },
          'Liverpool St': { x: 0.68, y: 0.30 },
          'Oxford Circus': { x: 0.36, y: 0.38 },
          'Bank': { x: 0.60, y: 0.40 },
          'Greenwich': { x: 0.75, y: 0.65 },
          'Heathrow': { x: 0.08, y: 0.52 },
          'Stratford': { x: 0.80, y: 0.22 },
          'Brixton': { x: 0.42, y: 0.72 },
          'Wimbledon': { x: 0.28, y: 0.78 },
        },
        edges: [
          ['Kings Cross', 'Euston', 2], ['Kings Cross', 'Liverpool St', 10], ['Kings Cross', 'Bank', 12],
          ['Euston', 'Oxford Circus', 4], ['Paddington', 'Oxford Circus', 6], ['Paddington', 'Heathrow', 22],
          ['Oxford Circus', 'Victoria', 8], ['Oxford Circus', 'Bank', 8], ['Victoria', 'Waterloo', 4],
          ['Victoria', 'Brixton', 6], ['Victoria', 'Wimbledon', 14], ['Waterloo', 'London Bridge', 3],
          ['London Bridge', 'Bank', 3], ['Bank', 'Liverpool St', 4], ['Bank', 'Canary Wharf', 10],
          ['Liverpool St', 'Stratford', 8], ['Canary Wharf', 'Greenwich', 8], ['Canary Wharf', 'Stratford', 6],
          ['Brixton', 'Wimbledon', 12], ['London Bridge', 'Greenwich', 10],
        ],
        startNode: 'Paddington', endNode: 'Canary Wharf'
      },
      tokyo: {
        nodes: {
          'Shinjuku': { x: 0.32, y: 0.34 },
          'Shibuya': { x: 0.35, y: 0.48 },
          'Akihabara': { x: 0.58, y: 0.28 },
          'Ueno': { x: 0.55, y: 0.18 },
          'Asakusa': { x: 0.68, y: 0.14 },
          'Ikebukuro': { x: 0.28, y: 0.18 },
          'Harajuku': { x: 0.28, y: 0.44 },
          'Ginza': { x: 0.60, y: 0.45 },
          'Tokyo Stn': { x: 0.55, y: 0.38 },
          'Odaiba': { x: 0.62, y: 0.68 },
          'Roppongi': { x: 0.42, y: 0.58 },
          'Shimokitazawa': { x: 0.22, y: 0.56 },
          'Nakameguro': { x: 0.34, y: 0.60 },
          'Yokohama': { x: 0.36, y: 0.88 },
          'Haneda': { x: 0.50, y: 0.88 },
        },
        edges: [
          ['Ikebukuro', 'Shinjuku', 10], ['Ikebukuro', 'Ueno', 12], ['Shinjuku', 'Harajuku', 5],
          ['Shinjuku', 'Shibuya', 8], ['Harajuku', 'Shibuya', 4], ['Shibuya', 'Nakameguro', 4],
          ['Shibuya', 'Roppongi', 7], ['Shibuya', 'Shimokitazawa', 6], ['Ueno', 'Akihabara', 5],
          ['Akihabara', 'Tokyo Stn', 6], ['Tokyo Stn', 'Ginza', 4], ['Tokyo Stn', 'Ueno', 8],
          ['Ginza', 'Roppongi', 9], ['Ginza', 'Odaiba', 10], ['Roppongi', 'Nakameguro', 8],
          ['Nakameguro', 'Yokohama', 22], ['Odaiba', 'Haneda', 14], ['Asakusa', 'Ueno', 6],
          ['Asakusa', 'Akihabara', 8], ['Yokohama', 'Haneda', 10],
        ],
        startNode: 'Ikebukuro', endNode: 'Odaiba'
      },
      newyork: {
        nodes: {
          'Times Square': { x: 0.40, y: 0.30 },
          'Central Park': { x: 0.35, y: 0.18 },
          'Brooklyn': { x: 0.60, y: 0.75 },
          'Queens': { x: 0.80, y: 0.30 },
          'Bronx': { x: 0.62, y: 0.08 },
          'Wall Street': { x: 0.50, y: 0.65 },
          'Harlem': { x: 0.44, y: 0.12 },
          'East Village': { x: 0.54, y: 0.48 },
          'Greenwich': { x: 0.44, y: 0.55 },
          'JFK Airport': { x: 0.88, y: 0.68 },
          'LGA Airport': { x: 0.82, y: 0.16 },
          'Hoboken': { x: 0.22, y: 0.52 },
          'Midtown': { x: 0.42, y: 0.40 },
          'Staten Island': { x: 0.28, y: 0.82 },
          'Brighton Beach': { x: 0.65, y: 0.90 },
        },
        edges: [
          ['Central Park', 'Harlem', 4], ['Central Park', 'Times Square', 8], ['Harlem', 'Bronx', 8],
          ['Bronx', 'LGA Airport', 14], ['LGA Airport', 'Queens', 10], ['Queens', 'JFK Airport', 16],
          ['Times Square', 'Midtown', 3], ['Midtown', 'East Village', 8], ['Midtown', 'Hoboken', 12],
          ['East Village', 'Wall Street', 10], ['East Village', 'Greenwich', 4], ['Greenwich', 'Hoboken', 8],
          ['Wall Street', 'Brooklyn', 10], ['Brooklyn', 'JFK Airport', 14], ['Brooklyn', 'Brighton Beach', 10],
          ['Brooklyn', 'Staten Island', 16], ['Staten Island', 'Hoboken', 18], ['Queens', 'Brooklyn', 12],
        ],
        startNode: 'Central Park', endNode: 'JFK Airport'
      }
    };

    // ═══════════════════════════════════════════════════════
    // TAB SWITCHING
    // ═══════════════════════════════════════════════════════
    function switchTab(name) {
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
      const idx = ['learn', 'city', 'stepper', 'grid'].indexOf(name);
      document.querySelectorAll('.tab')[idx].classList.add('active');
      document.getElementById('page-' + name).classList.add('active');
      if (name === 'grid' && typeof gridDraw === 'function') gridDraw();
    }

    // ═══════════════════════════════════════════════════════
    // PAGE 1: LEARN
    // ═══════════════════════════════════════════════════════
    function buildLearnPage() {
      const grid = document.getElementById('algo-cards');
      for (const [key, m] of Object.entries(ALGO_META)) {
        const card = document.createElement('div');
        card.className = 'algo-card';
        card.style.setProperty('--clr', m.color);
        card.innerHTML = `
      <div class="algo-name">${m.name}</div>
      <div class="algo-tag">${m.tag}</div>
      <div class="algo-desc">${m.desc}</div>
    `;
        card.onclick = () => showExplain(key);
        grid.appendChild(card);
      }
    }

    function showExplain(key) {
      document.querySelectorAll('.algo-card').forEach(c => c.classList.remove('selected'));
      const cards = document.querySelectorAll('.algo-card');
      const idx = Object.keys(ALGO_META).indexOf(key);
      cards[idx].classList.add('selected');

      const m = ALGO_META[key];
      const panel = document.getElementById('explain-panel');
      panel.className = 'explain-panel visible';
      panel.style.setProperty('--clr', m.color);
      panel.style.borderColor = m.color + '44';

      const stepsHtml = m.steps.map((s, i) => `
    <div class="estep">
      <div class="estep-num" style="background:${m.color}">${i + 1}</div>
      <div class="estep-text"><strong style="color:${m.color}">${s.t}:</strong> ${s.d}</div>
    </div>
  `).join('');

      panel.innerHTML = `
    <h2 style="color:${m.color}">${m.name} — How It Works</h2>
    <div style="display:flex;gap:12px;flex-wrap:wrap;margin-bottom:14px">
      <span style="font-size:12px;color:var(--muted)">✓ Complete: <strong style="color:var(--text)">${m.complete ? 'Yes' : 'No'}</strong></span>
      <span style="font-size:12px;color:var(--muted)">✓ Optimal: <strong style="color:var(--text)">${m.optimal ? 'Yes' : 'No'}</strong></span>
    </div>
    <div class="explain-steps">${stepsHtml}</div>
    <div class="complexity-row">
      <div class="cbox"><div class="cbox-label">Time Complexity</div><div class="cbox-val" style="color:${m.color}">${m.complexity.time}</div></div>
      <div class="cbox"><div class="cbox-label">Space Complexity</div><div class="cbox-val" style="color:${m.color}">${m.complexity.space}</div></div>
    </div>
  `;
      panel.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    }

    // ═══════════════════════════════════════════════════════
    // GRAPH / PRIORITY QUEUE UTILS
    // ═══════════════════════════════════════════════════════
    class MinHeap {
      constructor() { this.d = []; }
      push(v, p) {
        this.d.push({ v, p });
        let i = this.d.length - 1;
        while (i > 0) {
          const j = (i - 1) >> 1;
          if (this.d[j].p <= this.d[i].p) break;
          [this.d[j], this.d[i]] = [this.d[i], this.d[j]]; i = j;
        }
      }
      pop() {
        if (!this.d.length) return null;
        const top = this.d[0];
        const last = this.d.pop();
        if (this.d.length) {
          this.d[0] = last;
          let i = 0;
          for (; ;) {
            let s = i, a = 2 * i + 1, b = 2 * i + 2;
            if (a < this.d.length && this.d[a].p < this.d[s].p) s = a;
            if (b < this.d.length && this.d[b].p < this.d[s].p) s = b;
            if (s === i) break;
            [this.d[s], this.d[i]] = [this.d[i], this.d[s]]; i = s;
          }
        }
        return top;
      }
      get size() { return this.d.length; }
    }

    function buildAdj(edges, obstacles) {
      const adj = {};
      const blocked = obstacles || new Set();
      for (const [a, b, w] of edges) {
        if (!adj[a]) adj[a] = [];
        if (!adj[b]) adj[b] = [];
        // Only add edges to non-obstacle nodes
        if (!blocked.has(b)) adj[a].push({ n: b, w });
        if (!blocked.has(a)) adj[b].push({ n: a, w });
      }
      return adj;
    }

    function heuristic(nodes, a, b) {
      const na = nodes[a], nb = nodes[b];
      return Math.sqrt((na.x - nb.x) ** 2 + (na.y - nb.y) ** 2) * 100;
    }

    function recon(par, s, e) {
      const path = []; let cur = e;
      while (cur !== null && cur !== undefined) { path.unshift(cur); if (cur === s) break; cur = par[cur]; }
      return path[0] === s ? path : [];
    }

    // ── A* ──
    function runAstar(nodes, edges, start, end, obstacles) {
      const blocked = obstacles || cityState.obstacles;
      if (blocked.has(start) || blocked.has(end)) {
        return { visited: [], path: [], cost: '∞', trace: [{ node: start, action: 'visit', note: 'Start or End is an obstacle!' }] };
      }
      const adj = buildAdj(edges, blocked);
      const h = new MinHeap;
      const g = { [start]: 0 }, par = { [start]: null };
      const cl = new Set, visited = [], trace = [];
      h.push(start, heuristic(nodes, start, end));
      while (h.size) {
        const { v: cur } = h.pop();
        if (cl.has(cur)) continue;
        cl.add(cur); visited.push(cur);
        const f = (g[cur] || 0) + heuristic(nodes, cur, end);
        trace.push({ node: cur, action: 'visit', g: g[cur]?.toFixed(1), f: f.toFixed(1) });
        if (cur === end) { trace.push({ node: cur, action: 'goal', g: g[cur]?.toFixed(1) }); break; }
        for (const { n, w } of (adj[cur] || [])) {
          const ng = (g[cur] || 0) + w;
          if (ng < (g[n] ?? Infinity)) {
            g[n] = ng; par[n] = cur;
            const fn = ng + heuristic(nodes, n, end);
            h.push(n, fn);
            trace.push({ node: n, action: 'update', g: ng.toFixed(1), f: fn.toFixed(1) });
          }
        }
      }
      return { visited, path: recon(par, start, end), cost: g[end]?.toFixed(1) ?? '?', trace };
    }

    // ── Dijkstra ──
    function runDijkstra(nodes, edges, start, end, obstacles) {
      const blocked = obstacles || cityState.obstacles;
      if (blocked.has(start) || blocked.has(end)) {
        return { visited: [], path: [], cost: '∞', trace: [{ node: start, action: 'visit', note: 'Start or End is an obstacle!' }] };
      }
      const adj = buildAdj(edges, blocked);
      const h = new MinHeap;
      const dist = { [start]: 0 }, par = { [start]: null };
      const cl = new Set, visited = [], trace = [];
      h.push(start, 0);
      while (h.size) {
        const { v: cur } = h.pop();
        if (cl.has(cur)) continue;
        cl.add(cur); visited.push(cur);
        trace.push({ node: cur, action: 'visit', g: dist[cur]?.toFixed(1) });
        if (cur === end) { trace.push({ node: cur, action: 'goal', g: dist[cur]?.toFixed(1) }); break; }
        for (const { n, w } of (adj[cur] || [])) {
          const nd = (dist[cur] || 0) + w;
          if (nd < (dist[n] ?? Infinity)) {
            dist[n] = nd; par[n] = cur; h.push(n, nd);
            trace.push({ node: n, action: 'relax', g: nd.toFixed(1) });
          }
        }
      }
      return { visited, path: recon(par, start, end), cost: dist[end]?.toFixed(1) ?? '?', trace };
    }

    // ── BFS ──
    function runBfs(nodes, edges, start, end, obstacles) {
      const blocked = obstacles || cityState.obstacles;
      if (blocked.has(start) || blocked.has(end)) {
        return { visited: [], path: [], cost: '∞', trace: [{ node: start, action: 'dequeue', note: 'Start or End is an obstacle!' }] };
      }
      const adj = buildAdj(edges, blocked);
      const q = [start], par = { [start]: null }, seen = new Set([start]);
      const visited = [], trace = [];
      while (q.length) {
        const cur = q.shift(); visited.push(cur);
        trace.push({ node: cur, action: 'dequeue' });
        if (cur === end) { trace.push({ node: cur, action: 'goal' }); break; }
        for (const { n } of (adj[cur] || [])) {
          if (!seen.has(n)) { seen.add(n); par[n] = cur; q.push(n); trace.push({ node: n, action: 'enqueue' }); }
        }
      }
      // BFS path cost
      let cost = 0; const p = recon(par, start, end);
      const adj2 = buildAdj(edges, blocked);
      for (let i = 0; i < p.length - 1; i++) {
        const e = (adj2[p[i]] || []).find(x => x.n === p[i + 1]);
        if (e) cost += e.w;
      }
      return { visited, path: p, cost: cost.toFixed(1), trace };
    }

    // ── D* Lite (simplified backward A*) ──
    function runDstar(nodes, edges, start, end, obstacles) {
      const blocked = obstacles || cityState.obstacles;
      if (blocked.has(start) || blocked.has(end)) {
        return { visited: [], path: [], cost: '∞', trace: [{ node: start, action: 'visit', note: 'Start or End is an obstacle!' }] };
      }
      const adj = buildAdj(edges, blocked);
      const h = new MinHeap;
      const g = {}, par = {};
      g[end] = 0; par[end] = null;
      const cl = new Set, visited = [], trace = [];
      h.push(end, heuristic(nodes, end, start));
      while (h.size) {
        const { v: cur } = h.pop();
        if (cl.has(cur)) continue;
        cl.add(cur); visited.push(cur);
        trace.push({ node: cur, action: 'visit', g: g[cur].toFixed(1), note: 'backward from goal' });
        if (cur === start) { trace.push({ node: cur, action: 'goal', g: g[cur].toFixed(1) }); break; }
        for (const { n, w } of (adj[cur] || [])) {
          const ng = g[cur] + w;
          if (g[n] === undefined || ng < g[n]) {
            g[n] = ng; par[n] = cur;
            h.push(n, ng + heuristic(nodes, n, start));
            trace.push({ node: n, action: 'update', g: ng.toFixed(1) });
          }
        }
      }
      // Check if start is reachable from goal
      if (g[start] === undefined) {
        trace.push({ node: start, action: 'visit', note: 'UNREACHABLE — no path from goal to start' });
        return { visited, path: [], cost: '∞', trace };
      }
      // Build path forward: start → end via greedy g-value descent
      let cur = start; const path = [start];
      const pathSeen = new Set([start]);
      while (cur !== end) {
        const nbs = (adj[cur] || []).filter(x => g[x.n] !== undefined);
        if (!nbs.length) { path.length = 0; break; }
        nbs.sort((a, b) => g[a.n] - g[b.n]);
        const next = nbs[0].n;
        if (pathSeen.has(next)) { path.length = 0; break; }
        path.push(next); pathSeen.add(next); cur = next;
      }
      if (path.length === 0) {
        return { visited, path: [], cost: '∞', trace };
      }
      return { visited, path, cost: g[start].toFixed(1), trace };
    }

    // ═══════════════════════════════════════════════════════
    // PAGE 2: CITY SIMULATOR
    // ═══════════════════════════════════════════════════════
    let cityState = {
      graph: null,
      selectedAlgo: 'astar',
      start: null,
      end: null,
      result: null,
      animStep: 0,
      animTimer: null,
      graphMode: 'preset',
      manualEdges: [],
      manualNodes: {},
      obstacles: new Set()  // node names blocked as obstacles
    };

    let obstacleClickMode = false; // for preset mode obstacle toggling

    function buildCityAlgoBtns() {
      const wrap = document.getElementById('city-algo-btns');
      for (const [k, m] of Object.entries(ALGO_META)) {
        const b = document.createElement('button');
        b.className = 'sbtn' + (k === cityState.selectedAlgo ? ' active' : '');
        b.style.setProperty('--clr', m.color);
        b.textContent = m.name;
        b.onclick = () => { cityState.selectedAlgo = k; refreshCityAlgoBtns(); };
        wrap.appendChild(b);
      }
    }
    function refreshCityAlgoBtns() {
      document.querySelectorAll('#city-algo-btns .sbtn').forEach((b, i) => {
        const k = Object.keys(ALGO_META)[i];
        const m = ALGO_META[k];
        b.style.setProperty('--clr', m.color);
        b.className = 'sbtn' + (k === cityState.selectedAlgo ? ' active' : '');
      });
    }

    function loadCity() {
      const name = document.getElementById('city-select').value;
      cityState.graph = CITIES[name];
      cityState.start = cityState.graph.startNode;
      cityState.end = cityState.graph.endNode;
      cityState.result = null;
      document.getElementById('city-result').style.display = 'none';
      document.getElementById('trace-log').innerHTML = '';
      drawCity();
    }

    function setGraphMode(mode) {
      cityState.graphMode = mode;
      document.getElementById('tog-preset').className = 'ctog' + (mode === 'preset' ? ' on' : '');
      document.getElementById('tog-manual').className = 'ctog' + (mode === 'manual' ? ' on' : '');
      document.getElementById('preset-sel').style.display = mode === 'preset' ? '' : 'none';
      document.getElementById('manual-input').style.display = mode === 'manual' ? '' : 'none';
      document.getElementById('edge-table-wrap').style.display = mode === 'manual' ? '' : 'none';
      if (mode === 'manual') {
        cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: null, endNode: null };
        cityState.start = null; cityState.end = null; cityState.result = null;
        renderEdgeTable(); drawCity();
      } else {
        loadCity();
      }
    }

    function openEdgeModal() { document.getElementById('edge-modal').classList.add('open'); }
    function closeEdgeModal() { document.getElementById('edge-modal').classList.remove('open'); }

    function addManualEdge() {
      const from = document.getElementById('m-from').value.trim();
      const to = document.getElementById('m-to').value.trim();
      const dist = parseFloat(document.getElementById('m-dist').value);
      if (!from || !to || isNaN(dist) || dist <= 0) { alert('Fill all fields correctly.'); return; }
      cityState.manualEdges.push([from, to, dist]);
      // auto-place nodes if not existing
      if (!cityState.manualNodes[from]) cityState.manualNodes[from] = { x: 0.1 + Math.random() * 0.8, y: 0.1 + Math.random() * 0.8 };
      if (!cityState.manualNodes[to]) cityState.manualNodes[to] = { x: 0.1 + Math.random() * 0.8, y: 0.1 + Math.random() * 0.8 };
      cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: cityState.start, endNode: cityState.end };
      renderEdgeTable(); drawCity(); closeEdgeModal();
      document.getElementById('m-from').value = document.getElementById('m-to').value = '';
      document.getElementById('m-dist').value = '10';
    }

    function clearManual() {
      cityState.manualEdges = []; cityState.manualNodes = {};
      cityState.graph = { nodes: {}, edges: [], startNode: null, endNode: null };
      cityState.start = null; cityState.end = null; cityState.result = null;
      renderEdgeTable(); drawCity();
    }

    function renderEdgeTable() {
      const tb = document.getElementById('edge-tbody');
      tb.innerHTML = cityState.manualEdges.map((e, i) =>
        `<tr><td>${e[0]}</td><td>${e[1]}</td><td>${e[2]} km</td><td><button class="edge-del" onclick="removeEdge(${i})">✕</button></td></tr>`
      ).join('');
    }

    function removeEdge(i) {
      cityState.manualEdges.splice(i, 1);
      cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges };
      renderEdgeTable(); drawCity();
    }

    // ── Canvas drawing ──
    const cityCanvas = document.getElementById('city-canvas');
    const cityCx = cityCanvas.getContext('2d');

    function nodePos(node, W, H) {
      const n = cityState.graph?.nodes[node];
      if (!n) return null;
      return { x: n.x * W, y: n.y * H };
    }

    function drawCity() {
      const W = cityCanvas.width, H = cityCanvas.height;
      cityCx.clearRect(0, 0, W, H);
      cityCx.fillStyle = '#0c1018';
      cityCx.fillRect(0, 0, W, H);

      const g = cityState.graph;
      if (!g || !g.nodes || Object.keys(g.nodes).length === 0) {
        cityCx.fillStyle = '#2a4060';
        cityCx.font = '14px JetBrains Mono';
        cityCx.textAlign = 'center';
        cityCx.fillText('Add edges to build your graph', W / 2, H / 2);
        return;
      }

      const res = cityState.result;
      const visitedSet = new Set(res ? res.visited.slice(0, cityState.animStep) : []);
      const pathSet = new Set(res && cityState.animStep > res.visited.length ? res.path : []);

      // Draw edges
      for (const [a, b, w] of g.edges) {
        const pa = nodePos(a, W, H), pb = nodePos(b, W, H);
        if (!pa || !pb) continue;
        const isPath = res && pathSet.has(a) && pathSet.has(b) &&
          (res.path.indexOf(a) === res.path.indexOf(b) - 1 || res.path.indexOf(b) === res.path.indexOf(a) - 1);
        cityCx.strokeStyle = isPath ? '#ffcc0099' : '#1c2840';
        cityCx.lineWidth = isPath ? 3 : 1.5;
        cityCx.beginPath();
        cityCx.moveTo(pa.x, pa.y);
        cityCx.lineTo(pb.x, pb.y);
        cityCx.stroke();
        // edge weight label
        const mx = (pa.x + pb.x) / 2, my = (pa.y + pb.y) / 2;
        cityCx.fillStyle = isPath ? '#ffcc00cc' : '#2a4060';
        cityCx.font = '9px JetBrains Mono';
        cityCx.textAlign = 'center';
        cityCx.fillText(w, mx, my - 3);
      }

      // Draw nodes
      for (const [name, pos] of Object.entries(g.nodes)) {
        const px = pos.x * W, py = pos.y * H;
        const isObstacle = cityState.obstacles.has(name);
        let clr = '#1a2840', border = '#2a4060', textClr = '#4a6a8a';
        const isEdgeSel = (typeof edgeFirstNode !== 'undefined') && edgeFirstNode === name;

        if (isObstacle) {
          clr = '#2a0a0a'; border = '#e05060'; textClr = '#e05060';
        } else if (name === cityState.start) { clr = '#0d2a10'; border = '#20d080'; textClr = '#20d080'; }
        else if (name === cityState.end) { clr = '#2a0d10'; border = '#f05060'; textClr = '#f05060'; }
        else if (isEdgeSel) { clr = '#1a1840'; border = '#a080ff'; textClr = '#c0b0ff'; }
        else if (pathSet.has(name)) { clr = '#1a3818'; border = '#20d08080'; textClr = '#c0f0d0'; }
        else if (visitedSet.has(name)) { clr = '#0d1e38'; border = ALGO_META[cityState.selectedAlgo].color + '80'; textClr = '#6090c0'; }

        const r = 14;
        // Glow for edge-selected node
        if (isEdgeSel) {
          cityCx.beginPath();
          cityCx.arc(px, py, r + 6, 0, Math.PI * 2);
          cityCx.fillStyle = '#a080ff18';
          cityCx.fill();
          cityCx.strokeStyle = '#a080ff40';
          cityCx.lineWidth = 1;
          cityCx.stroke();
        }

        // Obstacle glow
        if (isObstacle) {
          cityCx.beginPath();
          cityCx.arc(px, py, r + 5, 0, Math.PI * 2);
          cityCx.fillStyle = '#e0506015';
          cityCx.fill();
          cityCx.strokeStyle = '#e0506030';
          cityCx.lineWidth = 1;
          cityCx.stroke();
        }

        cityCx.beginPath();
        cityCx.arc(px, py, r, 0, Math.PI * 2);
        cityCx.fillStyle = clr;
        cityCx.fill();
        cityCx.strokeStyle = border;
        cityCx.lineWidth = isObstacle ? 2.5 : (isEdgeSel ? 2.5 : 1.5);
        cityCx.stroke();

        // Obstacle cross-hatch pattern
        if (isObstacle) {
          cityCx.save();
          cityCx.beginPath();
          cityCx.arc(px, py, r - 1, 0, Math.PI * 2);
          cityCx.clip();
          cityCx.strokeStyle = '#e0506035';
          cityCx.lineWidth = 1;
          for (let d = -r; d < r; d += 5) {
            cityCx.beginPath();
            cityCx.moveTo(px + d - r, py - r);
            cityCx.lineTo(px + d + r, py + r);
            cityCx.stroke();
          }
          cityCx.restore();
        }

        // Node label
        const shortName = isObstacle ? '⛔' : name.split(' ').map(w => w[0]).join('').slice(0, 3);
        cityCx.fillStyle = textClr;
        cityCx.font = isObstacle ? '11px sans-serif' : 'bold 9px JetBrains Mono';
        cityCx.textAlign = 'center';
        cityCx.textBaseline = 'middle';
        cityCx.fillText(shortName, px, py);

        // Full name below
        cityCx.fillStyle = textClr + 'cc';
        cityCx.font = '9px Syne';
        cityCx.textBaseline = 'top';
        const displayName = isObstacle ? '🚫 ' + (name.length > 9 ? name.slice(0, 8) + '…' : name) : (name.length > 12 ? name.slice(0, 10) + '…' : name);
        cityCx.fillText(displayName, px, py + r + 2);
      }
    }

    // ═══════════════════════════════════════════════════════
    // CANVAS INTERACTION MODES
    // ═══════════════════════════════════════════════════════
    let canvasMode = 'select'; // 'build' or 'select'
    let dragNode = null;
    let edgeFirstNode = null;
    let nodeCounter = 1;

    function setCanvasMode(mode) {
      canvasMode = mode;
      edgeFirstNode = null;
      document.querySelectorAll('#mode-bar .mode-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === mode);
        if (b.dataset.mode === 'obstacle' && mode === 'obstacle') b.classList.add('obstacle-active');
        else b.classList.remove('obstacle-active');
      });
      cityCanvas.className = '';
      const buildHint = document.getElementById('build-hint');
      const obstacleHint = document.getElementById('obstacle-hint');
      if (mode === 'build') {
        cityCanvas.classList.add('canvas-mode-add');
        document.getElementById('canvas-hint').textContent = 'Click empty space to add node · Drag to move · Click two nodes to connect';
        if (buildHint) buildHint.style.display = '';
        if (obstacleHint) obstacleHint.style.display = 'none';
      } else if (mode === 'obstacle') {
        cityCanvas.classList.add('canvas-mode-obstacle');
        document.getElementById('canvas-hint').textContent = '🚧 Click any node to toggle it as an obstacle';
        if (buildHint) buildHint.style.display = 'none';
        if (obstacleHint) obstacleHint.style.display = '';
      } else {
        document.getElementById('canvas-hint').textContent = 'Click a node to set START, click another to set END';
        if (buildHint) buildHint.style.display = 'none';
        if (obstacleHint) obstacleHint.style.display = 'none';
      }
    }

    // ── Obstacle helpers ──
    function toggleObstacleMode() {
      obstacleClickMode = !obstacleClickMode;
      const btn = document.getElementById('obstacle-mode-toggle');
      if (obstacleClickMode) {
        btn.textContent = '🚧 Done Adding Obstacles';
        btn.style.color = '#ff6070';
        btn.style.borderColor = '#e05060';
        btn.style.background = '#e0506018';
        cityCanvas.classList.add('canvas-mode-obstacle');
        document.getElementById('canvas-hint').textContent = '🚧 Click any node to toggle it as an obstacle. Click "Done" when finished.';
      } else {
        btn.textContent = '🚧 Toggle Obstacles';
        btn.style.color = '#e05060';
        btn.style.borderColor = '#e0506030';
        btn.style.background = '';
        cityCanvas.classList.remove('canvas-mode-obstacle');
        document.getElementById('canvas-hint').textContent = 'Click a node to set START, click another to set END';
      }
      drawCity();
    }

    function clearObstacles() {
      cityState.obstacles.clear();
      obstacleClickMode = false;
      const btn = document.getElementById('obstacle-mode-toggle');
      if (btn) {
        btn.textContent = '🚧 Toggle Obstacles';
        btn.style.color = '#e05060';
        btn.style.borderColor = '#e0506030';
        btn.style.background = '';
      }
      cityCanvas.classList.remove('canvas-mode-obstacle');
      document.getElementById('canvas-hint').textContent = 'Obstacles cleared. Click a node to set START/END.';
      drawCity();
    }

    function getCanvasCoords(e) {
      const rect = cityCanvas.getBoundingClientRect();
      const sx = cityCanvas.width / rect.width;
      const sy = cityCanvas.height / rect.height;
      return { mx: (e.clientX - rect.left) * sx, my: (e.clientY - rect.top) * sy };
    }

    function findClosestNode(mx, my, threshold = 20) {
      const g = cityState.graph;
      if (!g || !g.nodes) return null;
      const W = cityCanvas.width, H = cityCanvas.height;
      let closest = null, minD = Infinity;
      for (const [name, pos] of Object.entries(g.nodes)) {
        const d = Math.hypot(pos.x * W - mx, pos.y * H - my);
        if (d < minD && d < threshold) { minD = d; closest = name; }
      }
      return closest;
    }

    // ── Mouse Down ──
    let wasDragging = false;
    cityCanvas.addEventListener('mousedown', function(e) {
      wasDragging = false;
      const { mx, my } = getCanvasCoords(e);
      const isManual = cityState.graphMode === 'manual';
      const activeMode = isManual ? canvasMode : 'select';
      if (activeMode === 'build') {
        const node = findClosestNode(mx, my);
        if (node) { dragNode = node; cityCanvas.style.cursor = 'grabbing'; }
      }
    });

    // ── Mouse Move (drag) ──
    cityCanvas.addEventListener('mousemove', function(e) {
      if (!dragNode) return;
      wasDragging = true;
      const { mx, my } = getCanvasCoords(e);
      const W = cityCanvas.width, H = cityCanvas.height;
      const nx = Math.max(0.03, Math.min(0.97, mx / W));
      const ny = Math.max(0.03, Math.min(0.97, my / H));
      cityState.manualNodes[dragNode] = { x: nx, y: ny };
      cityState.graph.nodes[dragNode] = { x: nx, y: ny };
      drawCity();
    });

    // ── Mouse Up ──
    cityCanvas.addEventListener('mouseup', function(e) {
      if (dragNode) { dragNode = null; cityCanvas.style.cursor = 'crosshair'; }
    });

    // ── Click ──
    cityCanvas.addEventListener('click', function(e) {
      if (wasDragging) { wasDragging = false; return; }
      const { mx, my } = getCanvasCoords(e);
      const g = cityState.graph;
      if (!g || !g.nodes) {
        if (cityState.graphMode === 'manual' && canvasMode === 'build') {
          cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: null, endNode: null };
        } else return;
      }
      const W = cityCanvas.width, H = cityCanvas.height;
      const isManual = cityState.graphMode === 'manual';
      const activeMode = isManual ? canvasMode : (obstacleClickMode ? 'obstacle' : 'select');
      const closest = findClosestNode(mx, my);

      // ── Obstacle Mode ──
      if (activeMode === 'obstacle') {
        if (!closest) return;
        // Can't set start/end as obstacle
        if (closest === cityState.start || closest === cityState.end) {
          document.getElementById('canvas-hint').textContent = `⚠ Can't block "${closest}" — it's your start or end node!`;
          return;
        }
        if (cityState.obstacles.has(closest)) {
          cityState.obstacles.delete(closest);
          document.getElementById('canvas-hint').textContent = `✓ Removed obstacle from "${closest}"`;
        } else {
          cityState.obstacles.add(closest);
          document.getElementById('canvas-hint').textContent = `🚧 "${closest}" is now an obstacle (${cityState.obstacles.size} total)`;
        }
        drawCity();
        return;
      }

      if (activeMode === 'build') {
        if (!closest) {
          // Click empty space → add node
          const name = prompt('Enter node name:', 'Node ' + nodeCounter);
          if (!name || !name.trim()) return;
          const trimmed = name.trim();
          if (cityState.manualNodes[trimmed]) { alert('Node already exists!'); return; }
          nodeCounter++;
          cityState.manualNodes[trimmed] = { x: mx / W, y: my / H };
          cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: cityState.start, endNode: cityState.end };
          document.getElementById('canvas-hint').textContent = `✓ Added "${trimmed}". Click another spot or click two nodes to connect.`;
          drawCity();
        } else {
          // Click existing node → start/complete edge
          if (!edgeFirstNode) {
            edgeFirstNode = closest;
            document.getElementById('canvas-hint').textContent = `Selected "${closest}" — now click another node to connect, or click empty space to add.`;
            drawCity();
          } else {
            if (edgeFirstNode === closest) {
              edgeFirstNode = null;
              document.getElementById('canvas-hint').textContent = 'Deselected. Click a node or empty space.';
              return;
            }
            const exists = cityState.manualEdges.some(e =>
              (e[0] === edgeFirstNode && e[1] === closest) || (e[0] === closest && e[1] === edgeFirstNode)
            );
            if (exists) {
              document.getElementById('canvas-hint').textContent = 'Edge already exists between these nodes.';
              edgeFirstNode = null; return;
            }
            const distStr = prompt(`Enter distance from "${edgeFirstNode}" to "${closest}" (km):`, '10');
            const dist = parseFloat(distStr);
            if (isNaN(dist) || dist <= 0) { edgeFirstNode = null; return; }
            cityState.manualEdges.push([edgeFirstNode, closest, dist]);
            cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: cityState.start, endNode: cityState.end };
            document.getElementById('canvas-hint').textContent = `✓ Connected "${edgeFirstNode}" ↔ "${closest}" (${dist} km)`;
            renderEdgeTable();
            edgeFirstNode = null;
            drawCity();
          }
        }
      } else {
        // Select mode — set start/end
        if (!closest) return;
        // Remove obstacle status when selecting as start/end
        if (cityState.obstacles.has(closest)) {
          cityState.obstacles.delete(closest);
        }
        if (!cityState.start || (cityState.start && cityState.end)) {
          cityState.start = closest; cityState.end = null; cityState.result = null;
          document.getElementById('canvas-hint').textContent = `Start: ${closest}. Now click END node.`;
        } else {
          cityState.end = closest; cityState.result = null;
          document.getElementById('canvas-hint').textContent = `Start: ${cityState.start} → End: ${cityState.end}. Click Run.`;
        }
        drawCity();
      }
    });

    // ── Right-click to delete node ──
    cityCanvas.addEventListener('contextmenu', function(e) {
      e.preventDefault();
      if (cityState.graphMode !== 'manual' || canvasMode !== 'build') return;
      const { mx, my } = getCanvasCoords(e);
      const closest = findClosestNode(mx, my);
      if (!closest) return;
      if (!confirm(`Delete node "${closest}" and all its edges?`)) return;
      // Remove node
      delete cityState.manualNodes[closest];
      // Remove all edges connected to this node
      cityState.manualEdges = cityState.manualEdges.filter(e => e[0] !== closest && e[1] !== closest);
      // Clear start/end if deleted
      if (cityState.start === closest) cityState.start = null;
      if (cityState.end === closest) cityState.end = null;
      if (edgeFirstNode === closest) edgeFirstNode = null;
      cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: cityState.start, endNode: cityState.end };
      document.getElementById('canvas-hint').textContent = `✓ Deleted "${closest}" and its edges.`;
      renderEdgeTable();
      drawCity();
    });

    // ═══════════════════════════════════════════════════════
    // COMPARE ALGORITHMS
    // ═══════════════════════════════════════════════════════
    function compareAlgorithms() {
      const g = cityState.graph;
      if (!g || !cityState.start || !cityState.end) {
        alert('Select a start and end node first.'); return;
      }
      clearInterval(cityState.animTimer);
      const runners = { astar: runAstar, dijkstra: runDijkstra, bfs: runBfs, dstar: runDstar };
      const results = {};
      let bestCost = Infinity, bestAlgo = '';

      for (const [key, runner] of Object.entries(runners)) {
        const t0 = performance.now();
        const res = runner(g.nodes, g.edges, cityState.start, cityState.end);
        const t1 = performance.now();
        const cost = parseFloat(res.cost);
        results[key] = { ...res, time: (t1 - t0).toFixed(2) };
        if (!isNaN(cost) && cost < bestCost && res.path.length > 0) {
          bestCost = cost; bestAlgo = key;
        }
      }

      let rows = '';
      for (const [key, res] of Object.entries(results)) {
        const m = ALGO_META[key];
        const isBest = key === bestAlgo;
        rows += `<tr class="${isBest ? 'winner' : ''}" style="color:${m.color}">
          <td style="font-weight:700">${isBest ? '🏆 ' : ''}${m.name}</td>
          <td>${res.cost === '∞' || res.cost === '?' ? '∞' : res.cost + ' km'}</td>
          <td>${res.visited.length}</td>
          <td>${res.path.length > 0 ? res.path.length - 1 : '—'}</td>
          <td>${res.time} ms</td>
          <td>${res.path.length > 0 ? '✓' : '✕ No path'}</td>
        </tr>`;
      }

      // Build the best result details
      let bestHtml = '';
      if (bestAlgo) {
        const best = results[bestAlgo];
        const bm = ALGO_META[bestAlgo];
        bestHtml = `
          <div style="margin-top:14px;padding:12px;background:var(--bg3);border:1px solid ${bm.color}40;border-radius:10px">
            <div style="font-size:13px;font-weight:700;color:${bm.color};margin-bottom:8px">🏆 Best: ${bm.name}</div>
            <div class="result-row">
              <div class="rbox"><div class="rbox-label">Distance</div><div class="rbox-val">${best.cost} km</div></div>
              <div class="rbox"><div class="rbox-label">Visited</div><div class="rbox-val">${best.visited.length}</div></div>
              <div class="rbox"><div class="rbox-label">Hops</div><div class="rbox-val">${best.path.length - 1}</div></div>
            </div>
            <div style="font-size:11px;color:var(--muted);margin-top:10px;font-family:'JetBrains Mono',monospace;line-height:1.8">
              ${best.path.join(' → ')}
            </div>
          </div>`;
      } else {
        bestHtml = `<div style="margin-top:12px;color:#e05060;font-size:13px;font-weight:600">✕ No algorithm found a path from ${cityState.start} to ${cityState.end}</div>`;
      }

      const panel = document.getElementById('compare-panel');
      panel.classList.add('open');
      document.getElementById('compare-body').innerHTML = `
        <table class="compare-table">
          <thead><tr><th>Algorithm</th><th>Distance</th><th>Nodes Visited</th><th>Hops</th><th>Time</th><th>Status</th></tr></thead>
          <tbody>${rows}</tbody>
        </table>
        <div style="font-size:11px;color:var(--muted);margin-top:8px;font-family:'JetBrains Mono',monospace">
          Route: ${cityState.start} → ${cityState.end}
        </div>
        ${bestHtml}
      `;

      // Auto-visualize best algorithm path
      if (bestAlgo) {
        cityState.selectedAlgo = bestAlgo;
        refreshCityAlgoBtns();
        const best = results[bestAlgo];
        cityState.result = best;
        cityState.animStep = best.visited.length + best.path.length;
        drawCity();
      }
    }

    function runCity() {
      const g = cityState.graph;
      if (!g || !cityState.start || !cityState.end) {
        alert('Select a start and end node first.'); return;
      }
      clearInterval(cityState.animTimer);
      const algo = cityState.selectedAlgo;
      const runners = { astar: runAstar, dijkstra: runDijkstra, bfs: runBfs, dstar: runDstar };
      const res = runners[algo](g.nodes, g.edges, cityState.start, cityState.end);
      cityState.result = res;
      cityState.animStep = 0;

      // Trace log
      const log = document.getElementById('trace-log');
      log.innerHTML = '';
      res.trace.forEach((t, i) => {
        const div = document.createElement('div');
        div.className = 'tl-entry' + (t.action === 'goal' ? ' success' : t.action === 'visit' ? ' highlight' : '');
        const icons = { visit: '→', update: '↻', relax: '↓', goal: '★', enqueue: '+', dequeue: '◉' };
        const icon = icons[t.action] || '·';
        div.textContent = `[${i + 1}] ${icon} ${t.node}` + (t.g !== undefined ? ` (cost: ${t.g}${t.f !== undefined ? ', f: ' + t.f : ''})` : '') + (t.note ? ` — ${t.note}` : '');
        log.appendChild(div);
      });

      // Show result
      const total = res.visited.length + res.path.length;
      const speed = 11 - parseInt(document.getElementById('city-speed').value);
      cityState.animTimer = setInterval(() => {
        if (cityState.animStep < total) { cityState.animStep++; drawCity(); }
        else {
          clearInterval(cityState.animTimer);
          showCityResult(res, algo);
        }
      }, speed * 80);
    }

    function showCityResult(res, algo) {
      const panel = document.getElementById('city-result');
      panel.style.display = 'block';
      const m = ALGO_META[algo];
      document.getElementById('city-result-body').innerHTML = `
    <div style="color:${m.color};font-size:12px;font-weight:700;margin-bottom:10px">${m.name}</div>
    <div class="result-row">
      <div class="rbox"><div class="rbox-label">Distance</div><div class="rbox-val">${res.cost} km</div></div>
      <div class="rbox"><div class="rbox-label">Nodes Visited</div><div class="rbox-val">${res.visited.length}</div></div>
      <div class="rbox"><div class="rbox-label">Path Hops</div><div class="rbox-val">${res.path.length > 0 ? res.path.length - 1 : '—'}</div></div>
    </div>
    ${res.path.length > 0 ? `<div style="font-size:11px;color:var(--muted);margin-top:10px;font-family:'JetBrains Mono',monospace;line-height:1.8">${res.path.join(' → ')}</div>` : '<div style="color:#e05060;font-size:12px;margin-top:8px">No path found</div>'}
  `;
    }

    function resetCity() {
      clearInterval(cityState.animTimer);
      cityState.result = null; cityState.animStep = 0;
      document.getElementById('city-result').style.display = 'none';
      document.getElementById('trace-log').innerHTML = '';
      drawCity();
    }

    function clearCity() {
      resetCity();
      cityState.start = null; cityState.end = null;
      document.getElementById('canvas-hint').textContent = 'Click a node to set START, click another to set END';
      drawCity();
    }

    document.getElementById('city-speed').oninput = function () {
      document.getElementById('speed-lbl').textContent = this.value;
    };

    // ═══════════════════════════════════════════════════════
    // PAGE 3: STEP-BY-STEP STEPPER
    // ═══════════════════════════════════════════════════════
    let stepperState = {
      steps: [],
      currentStep: 0,
      algo: 'astar',
      graph: null,
    };

    function buildStepperAlgoBtns() {
      const wrap = document.getElementById('step-algo-btns');
      for (const [k, m] of Object.entries(ALGO_META)) {
        const b = document.createElement('button');
        b.id = 'sab_' + k;
        b.className = 'sbtn' + (k === stepperState.algo ? ' active' : '');
        b.style.setProperty('--clr', m.color);
        b.style.cssText += 'width:auto;padding:7px 14px;margin:0';
        b.textContent = m.name;
        b.onclick = () => { stepperState.algo = k; refreshStepAlgoBtns(); };
        wrap.appendChild(b);
      }
    }
    function refreshStepAlgoBtns() {
      for (const [k] of Object.entries(ALGO_META)) {
        const b = document.getElementById('sab_' + k);
        if (b) {
          const m = ALGO_META[k];
          b.style.setProperty('--clr', m.color);
          b.className = 'sbtn' + (k === stepperState.algo ? ' active' : '');
          b.style.cssText += 'width:auto;padding:7px 14px;margin:0';
        }
      }
    }

    function loadStepper() {
      const city = CITIES[document.getElementById('step-city').value];
      stepperState.graph = city;
      stepperState.steps = [];
      stepperState.currentStep = 0;
      document.getElementById('step-title-text').textContent = 'Select algorithm and click Generate';
      document.getElementById('step-desc-text').textContent = 'The full trace will appear here.';
      document.getElementById('step-counter').textContent = '–';
      document.getElementById('btn-prev').disabled = true;
      document.getElementById('btn-next').disabled = true;
      document.getElementById('state-display').textContent = 'State will appear here…';
      drawStepper();
    }

    function buildSteps() {
      const g = stepperState.graph;
      if (!g) { loadStepper(); return; }
      const algo = stepperState.algo;
      const runners = { astar: runAstar, dijkstra: runDijkstra, bfs: runBfs, dstar: runDstar };
      const res = runners[algo](g.nodes, g.edges, g.startNode, g.endNode);

      // Build detailed steps from trace
      const steps = [];
      const visitedSoFar = [], pathFound = [];
      const algoStepDefs = ALGO_META[algo].steps;
      let algoStepIdx = 0;

      steps.push({
        title: `Initialize — ${ALGO_META[algo].name}`,
        desc: algoStepDefs[0]?.d || 'Set up data structures.',
        visited: [],
        path: [],
        highlight: null,
        state: `Queue: [${g.startNode}]\ng[${g.startNode}] = 0`,
        algoPhase: 0
      });

      res.trace.forEach((t, i) => {
        const prev = steps[steps.length - 1];
        const newVisited = [...prev.visited];
        const newPath = [...prev.path];
        if (t.action === 'visit' || t.action === 'dequeue') newVisited.push(t.node);
        if (t.action === 'goal') {
          res.path.forEach(n => newPath.push(n));
        }
        let phase = t.action === 'goal' ? algoStepDefs.length - 1 :
          t.action === 'visit' || t.action === 'dequeue' ? 1 :
            t.action === 'update' || t.action === 'relax' || t.action === 'enqueue' ? 3 : 1;

        const phaseInfo = algoStepDefs[Math.min(phase, algoStepDefs.length - 1)];
        const icons = { visit: '→ Visiting', update: '↻ Updating', relax: '↓ Relaxing edge to', goal: '★ GOAL FOUND at', enqueue: '+ Enqueuing', dequeue: '◉ Processing' };
        steps.push({
          title: `${icons[t.action] || '·'} "${t.node}"`,
          desc: phaseInfo?.d || '',
          visited: newVisited,
          path: newPath,
          highlight: t.node,
          state: buildStateText(t, res, i),
          algoPhase: phase
        });
      });

      if (res.path.length > 0) {
        steps.push({
          title: '✓ Path Found!',
          desc: `Optimal path from <strong>${g.startNode}</strong> to <strong>${g.endNode}</strong> has been found. Total cost: ${res.cost} km over ${res.path.length - 1} hops.`,
          visited: res.visited,
          path: res.path,
          highlight: g.endNode,
          state: `Path: ${res.path.join(' → ')}\nTotal cost: ${res.cost} km\nNodes explored: ${res.visited.length}`,
          algoPhase: algoStepDefs.length - 1
        });
      }

      stepperState.steps = steps;
      stepperState.currentStep = 0;
      document.getElementById('btn-prev').disabled = true;
      document.getElementById('btn-next').disabled = steps.length <= 1;
      document.getElementById('step-counter').textContent = `1 / ${steps.length}`;
      renderStepperStep();
    }

    function buildStateText(t, res, idx) {
      const traceUpTo = res.trace.slice(0, idx + 1);
      const visited = traceUpTo.filter(x => x.action === 'visit' || x.action === 'dequeue').map(x => x.node);
      const inQueue = traceUpTo.filter(x => x.action === 'update' || x.action === 'relax' || x.action === 'enqueue').map(x => x.node).filter(n => !visited.includes(n));
      let s = `Processing: ${t.node}\n`;
      if (t.g !== undefined) s += `Cost g[${t.node}] = ${t.g}\n`;
      if (t.f !== undefined) s += `f[${t.node}] = ${t.f}\n`;
      s += `\nVisited (${visited.length}): ${visited.slice(-4).join(', ')}${visited.length > 4 ? '…' : ''}\n`;
      s += `Queue (~${inQueue.length}): ${[...new Set(inQueue)].slice(-3).join(', ')}`;
      return s;
    }

    function renderStepperStep() {
      const s = stepperState.steps[stepperState.currentStep];
      if (!s) return;
      document.getElementById('step-title-text').innerHTML = s.title;
      document.getElementById('step-desc-text').innerHTML = s.desc;
      document.getElementById('state-display').style.whiteSpace = 'pre-wrap';
      document.getElementById('state-display').textContent = s.state;
      document.getElementById('step-counter').textContent = `${stepperState.currentStep + 1} / ${stepperState.steps.length}`;
      document.getElementById('btn-prev').disabled = stepperState.currentStep === 0;
      document.getElementById('btn-next').disabled = stepperState.currentStep >= stepperState.steps.length - 1;
      drawStepper(s);
    }

    function goPrev() {
      if (stepperState.currentStep > 0) { stepperState.currentStep--; renderStepperStep(); }
    }
    function goNext() {
      if (stepperState.currentStep < stepperState.steps.length - 1) { stepperState.currentStep++; renderStepperStep(); }
    }

    const stepCanvas = document.getElementById('step-canvas');
    const stepCx = stepCanvas.getContext('2d');

    function drawStepper(step) {
      const W = stepCanvas.width, H = stepCanvas.height;
      stepCx.clearRect(0, 0, W, H);
      stepCx.fillStyle = '#0c1018';
      stepCx.fillRect(0, 0, W, H);

      const g = stepperState.graph;
      if (!g) return;

      const visitedSet = new Set(step ? step.visited : []);
      const pathSet = new Set(step ? step.path : []);
      const highlight = step ? step.highlight : null;
      const algo = stepperState.algo;
      const algoColor = ALGO_META[algo].color;

      // Edges
      for (const [a, b, w] of g.edges) {
        const pa = { x: g.nodes[a].x * W, y: g.nodes[a].y * H };
        const pb = { x: g.nodes[b].x * W, y: g.nodes[b].y * H };
        const isPath = pathSet.has(a) && pathSet.has(b);
        stepCx.strokeStyle = isPath ? '#ffcc0080' : '#1c2840';
        stepCx.lineWidth = isPath ? 3 : 1.5;
        stepCx.beginPath();
        stepCx.moveTo(pa.x, pa.y);
        stepCx.lineTo(pb.x, pb.y);
        stepCx.stroke();
        // weight
        const mx = (pa.x + pb.x) / 2, my = (pa.y + pb.y) / 2;
        stepCx.fillStyle = isPath ? '#ffcc00cc' : '#243050';
        stepCx.font = '9px JetBrains Mono';
        stepCx.textAlign = 'center';
        stepCx.fillText(w, mx, my - 3);
      }

      // Nodes
      for (const [name, pos] of Object.entries(g.nodes)) {
        const px = pos.x * W, py = pos.y * H;
        const isStart = name === g.startNode;
        const isEnd = name === g.endNode;
        const isHighlight = name === highlight;
        const isPath2 = pathSet.has(name);
        const isVisited = visitedSet.has(name);

        let clr = '#1a2840', border = '#2a4060', textClr = '#4a6a8a', r = 15;
        if (isStart) { clr = '#0d2a10'; border = '#20d080'; textClr = '#20d080'; }
        else if (isEnd) { clr = '#2a0d10'; border = '#f05060'; textClr = '#f05060'; }
        else if (isHighlight) { clr = algoColor + '33'; border = algoColor; textClr = algoColor; r = 18; }
        else if (isPath2) { clr = '#1a3818'; border = '#20d08060'; textClr = '#a0d0b0'; }
        else if (isVisited) { clr = algoColor + '15'; border = algoColor + '50'; textClr = algoColor + 'cc'; }

        if (isHighlight) {
          stepCx.beginPath();
          stepCx.arc(px, py, r + 6, 0, Math.PI * 2);
          stepCx.fillStyle = algoColor + '15';
          stepCx.fill();
        }

        stepCx.beginPath();
        stepCx.arc(px, py, r, 0, Math.PI * 2);
        stepCx.fillStyle = clr;
        stepCx.fill();
        stepCx.strokeStyle = border;
        stepCx.lineWidth = isHighlight ? 2 : 1.5;
        stepCx.stroke();

        const short = name.split(' ').map(w => w[0]).join('').slice(0, 3);
        stepCx.fillStyle = textClr;
        stepCx.font = `bold 9px JetBrains Mono`;
        stepCx.textAlign = 'center';
        stepCx.textBaseline = 'middle';
        stepCx.fillText(short, px, py);

        stepCx.fillStyle = textClr + 'cc';
        stepCx.font = '9px Syne';
        stepCx.textBaseline = 'top';
        const label = name.length > 12 ? name.slice(0, 10) + '…' : name;
        stepCx.fillText(label, px, py + r + 2);
      }
    }

    // ═══════════════════════════════════════════════════════
    // SAVE / LOAD SCENARIOS
    // ═══════════════════════════════════════════════════════
    const SAVE_KEY = 'algolab_scenarios';
    const MAX_SLOTS = 3;

    function getScenarioState() {
      return {
        graphMode: cityState.graphMode,
        citySelect: document.getElementById('city-select')?.value || 'chennai',
        manualNodes: { ...cityState.manualNodes },
        manualEdges: [...cityState.manualEdges],
        obstacles: [...cityState.obstacles],
        start: cityState.start,
        end: cityState.end,
        selectedAlgo: cityState.selectedAlgo,
        timestamp: new Date().toISOString()
      };
    }

    function applyScenarioState(data) {
      // Restore graph mode
      cityState.graphMode = data.graphMode || 'preset';
      cityState.manualNodes = data.manualNodes || {};
      cityState.manualEdges = data.manualEdges || [];
      cityState.obstacles = new Set(data.obstacles || []);
      cityState.start = data.start || null;
      cityState.end = data.end || null;
      cityState.selectedAlgo = data.selectedAlgo || 'astar';
      cityState.result = null;

      if (cityState.graphMode === 'preset') {
        document.getElementById('city-select').value = data.citySelect || 'chennai';
        const name = data.citySelect || 'chennai';
        cityState.graph = CITIES[name];
        // Restore custom start/end over preset defaults
        if (data.start) cityState.start = data.start;
        if (data.end) cityState.end = data.end;
      } else {
        cityState.graph = { nodes: cityState.manualNodes, edges: cityState.manualEdges, startNode: cityState.start, endNode: cityState.end };
      }

      // Update UI toggles
      setGraphMode(cityState.graphMode);
      refreshCityAlgoBtns();

      // Reset obstacle mode toggle
      obstacleClickMode = false;
      const btn = document.getElementById('obstacle-mode-toggle');
      if (btn) {
        btn.textContent = '\ud83d\udea7 Toggle Obstacles';
        btn.style.color = '#e05060';
        btn.style.borderColor = '#e0506030';
        btn.style.background = '';
      }

      document.getElementById('city-result').style.display = 'none';
      document.getElementById('trace-log').innerHTML = '';

      const obsCount = cityState.obstacles.size;
      document.getElementById('canvas-hint').textContent = 
        `✓ Scenario loaded! ${obsCount > 0 ? obsCount + ' obstacle(s) restored. ' : ''}` +
        (cityState.start && cityState.end ? `${cityState.start} → ${cityState.end}` : 'Set START and END nodes.');

      drawCity();
    }

    function getSavedSlots() {
      try {
        return JSON.parse(localStorage.getItem(SAVE_KEY)) || {};
      } catch { return {}; }
    }

    function renderSaveSlots() {
      const wrap = document.getElementById('save-slots');
      if (!wrap) return;
      const saved = getSavedSlots();
      let html = '';
      for (let i = 1; i <= MAX_SLOTS; i++) {
        const key = 'slot' + i;
        const data = saved[key];
        const hasData = !!data;
        const label = hasData
          ? `Slot ${i}: ${data.graphMode === 'preset' ? (data.citySelect || '?').toUpperCase() : 'Manual'} · ${(data.obstacles || []).length}⛔ · ${new Date(data.timestamp).toLocaleTimeString()}`
          : `Slot ${i}: (empty)`;
        html += `
          <div class="save-slot">
            <span class="slot-name ${hasData ? 'has-data' : ''}" title="${label}">${label}</span>
            <button class="slot-btn save" onclick="saveToSlot(${i})" title="Save current state">💾</button>
            <button class="slot-btn load" onclick="loadFromSlot(${i})" ${hasData ? '' : 'disabled'} title="Load">📂</button>
            <button class="slot-btn del" onclick="deleteSlot(${i})" ${hasData ? '' : 'disabled'} title="Delete">✕</button>
          </div>`;
      }
      wrap.innerHTML = html;
    }

    function saveToSlot(i) {
      const saved = getSavedSlots();
      saved['slot' + i] = getScenarioState();
      localStorage.setItem(SAVE_KEY, JSON.stringify(saved));
      renderSaveSlots();
      document.getElementById('canvas-hint').textContent = `💾 Scenario saved to Slot ${i}!`;
    }

    function loadFromSlot(i) {
      const saved = getSavedSlots();
      const data = saved['slot' + i];
      if (!data) { alert('Slot is empty.'); return; }
      applyScenarioState(data);
      renderSaveSlots();
    }

    function deleteSlot(i) {
      if (!confirm(`Delete Slot ${i}?`)) return;
      const saved = getSavedSlots();
      delete saved['slot' + i];
      localStorage.setItem(SAVE_KEY, JSON.stringify(saved));
      renderSaveSlots();
      document.getElementById('canvas-hint').textContent = `Slot ${i} deleted.`;
    }

    function exportScenario() {
      const data = getScenarioState();
      const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      const mode = data.graphMode === 'preset' ? data.citySelect : 'manual';
      const obs = (data.obstacles || []).length;
      a.download = `scenario_${mode}_${obs}obs_${Date.now()}.json`;
      a.click();
      URL.revokeObjectURL(url);
      document.getElementById('canvas-hint').textContent = '📤 Scenario exported as JSON file!';
    }

    function importScenario(event) {
      const file = event.target.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = function(e) {
        try {
          const data = JSON.parse(e.target.result);
          if (!data.graphMode) throw new Error('Invalid scenario file');
          applyScenarioState(data);
          document.getElementById('canvas-hint').textContent = `📥 Scenario loaded from "${file.name}"!`;
        } catch (err) {
          alert('Failed to load scenario: ' + err.message);
        }
      };
      reader.readAsText(file);
      event.target.value = ''; // reset so same file can be re-imported
    }

    // ═══════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════
    buildLearnPage();
    buildCityAlgoBtns();
    buildStepperAlgoBtns();
    loadCity();
    loadStepper();
    renderSaveSlots();

    // ═══════════════════════════════════════════════════════
    // PAGE 4: GRID SIMULATOR (from lanja.html)
    // ═══════════════════════════════════════════════════════
    ;(function() {
      const ROWS = 26, COLS = 26, CS = 18;
      const road  = (r, c) => r % 5 === 0 || c % 5 === 0;
      const inter = (r, c) => r % 5 === 0 && c % 5 === 0;
      const GK  = (r, c) => `${r},${c}`;
      const GFK = k => k.split(',').map(Number);
      const GMAN = (r1,c1,r2,c2) => Math.abs(r1-r2)+Math.abs(c1-c2);

      class GMH {
        constructor() { this.d = []; }
        push(v, p) {
          this.d.push({v, p}); let i = this.d.length - 1;
          while (i > 0) { const j = (i-1) >> 1; if (this.d[j].p <= this.d[i].p) break; [this.d[j], this.d[i]] = [this.d[i], this.d[j]]; i = j; }
        }
        pop() {
          if (!this.d.length) return;
          const top = this.d[0]; const last = this.d.pop();
          if (this.d.length) { this.d[0] = last; let i = 0; for (;;) { let s=i, a=2*i+1, b=2*i+2; if (a<this.d.length&&this.d[a].p<this.d[s].p) s=a; if (b<this.d.length&&this.d[b].p<this.d[s].p) s=b; if (s===i) break; [this.d[s],this.d[i]]=[this.d[i],this.d[s]]; i=s; } }
          return top.v;
        }
        get size() { return this.d.length; }
      }

      function gnbrs(r, c, obs) {
        return [[-1,0],[1,0],[0,-1],[0,1]].map(([dr,dc])=>[r+dr,c+dc]).filter(([nr,nc])=>nr>=0&&nr<ROWS&&nc>=0&&nc<COLS&&road(nr,nc)&&!obs.has(GK(nr,nc)));
      }
      function grecon(par, ek) { const p=[]; let cur=ek; while(cur!==null){p.unshift(cur);cur=par[cur];} return p; }

      function g_astar(s,e,obs) {
        const [sr,sc]=s,[er,ec]=e,sk=GK(sr,sc),ek=GK(er,ec);
        const h=new GMH,g={[sk]:0},par={[sk]:null},cl=new Set,vis=[];
        h.push(sk,GMAN(sr,sc,er,ec));
        while(h.size){const cur=h.pop();if(cl.has(cur))continue;cl.add(cur);vis.push(cur);if(cur===ek)break;const[r,c]=GFK(cur);for(const[nr,nc]of gnbrs(r,c,obs)){const nk=GK(nr,nc),ng=g[cur]+1;if(ng<(g[nk]??Infinity)){g[nk]=ng;par[nk]=cur;h.push(nk,ng+GMAN(nr,nc,er,ec));}}}
        return{vis,path:par[ek]!==undefined?grecon(par,ek):[]};
      }
      function g_dijkstra(s,e,obs) {
        const [sr,sc]=s,[er,ec]=e,sk=GK(sr,sc),ek=GK(er,ec);
        const h=new GMH,d={[sk]:0},par={[sk]:null},cl=new Set,vis=[];
        h.push(sk,0);
        while(h.size){const cur=h.pop();if(cl.has(cur))continue;cl.add(cur);vis.push(cur);if(cur===ek)break;const[r,c]=GFK(cur);for(const[nr,nc]of gnbrs(r,c,obs)){const nk=GK(nr,nc),nd=d[cur]+1;if(nd<(d[nk]??Infinity)){d[nk]=nd;par[nk]=cur;h.push(nk,nd);}}}
        return{vis,path:d[ek]!==undefined?grecon(par,ek):[]};
      }
      function g_bfs(s,e,obs) {
        const [sr,sc]=s,[er,ec]=e,sk=GK(sr,sc),ek=GK(er,ec);
        const q=[sk],par={[sk]:null},seen=new Set([sk]),vis=[];
        while(q.length){const cur=q.shift();vis.push(cur);if(cur===ek)break;const[r,c]=GFK(cur);for(const[nr,nc]of gnbrs(r,c,obs)){const nk=GK(nr,nc);if(!seen.has(nk)){seen.add(nk);par[nk]=cur;q.push(nk);}}}
        return{vis,path:par[ek]!==undefined?grecon(par,ek):[]};
      }
      function g_dstar(s,e,obs) {
        const [sr,sc]=s,[er,ec]=e,sk=GK(sr,sc),ek=GK(er,ec);
        const h=new GMH,g={[ek]:0},par={[ek]:null},cl=new Set,vis=[];
        h.push(ek,GMAN(er,ec,sr,sc));
        while(h.size){const cur=h.pop();if(cl.has(cur))continue;cl.add(cur);vis.push(cur);if(cur===sk)break;const[r,c]=GFK(cur);for(const[nr,nc]of gnbrs(r,c,obs)){const nk=GK(nr,nc),ng=g[cur]+1;if(ng<(g[nk]??Infinity)){g[nk]=ng;par[nk]=cur;h.push(nk,ng+GMAN(nr,nc,sr,sc));}}}
        if(par[sk]===undefined)return{vis,path:[]};
        let cur2=sk;const path=[sk];while(cur2!==ek){const nx=par[cur2];if(!nx)break;path.push(nx);cur2=nx;}
        return{vis,path};
      }

      const GALGOS={astar:g_astar,dijkstra:g_dijkstra,bfs:g_bfs,dstar:g_dstar};
      const GALGO_INFO={
        astar:{name:'A* Search',color:'#3a8ff0',visited:'#3a8ff030'},
        dijkstra:{name:"Dijkstra's",color:'#f0a030',visited:'#f0a03030'},
        bfs:{name:'BFS',color:'#a060f0',visited:'#a060f030'},
        dstar:{name:'D* Lite',color:'#20d080',visited:'#20d08030'},
      };

      const gst = {
        obs:new Set(), start:[0,0], end:[25,25],
        mode:'obstacle', algo:'astar', result:null,
        step:0, running:false, speed:30, dragging:false, tlPhase:0
      };

      const gcv = document.getElementById('grid-canvas');
      const gcx = gcv.getContext('2d');

      function gDrawBuilding(x,y) {
        gcx.fillStyle='#0e1525';gcx.fillRect(x,y,CS,CS);
        gcx.fillStyle='#141d32';gcx.fillRect(x+1,y+2,CS-2,CS-3);
        gcx.fillStyle='#1a2a44';gcx.fillRect(x+1,y+1,CS-2,3);
        const wc=['#243060','#1c2650','#2a3878'];
        for(let wr=0;wr<2;wr++)for(let wc2=0;wc2<2;wc2++){gcx.fillStyle=wc[(wr+wc2)%3];gcx.fillRect(x+2+wc2*7,y+5+wr*5,4,3);}
      }
      function gDrawRoad(r,c,x,y) {
        gcx.fillStyle=inter(r,c)?'#181828':'#131220';gcx.fillRect(x,y,CS,CS);
        if(!inter(r,c)){gcx.fillStyle='#1e1e34';gcx.fillRect(x+CS/2-1,y+CS/2-1,2,2);}
        else{const phases=[['#cc3333','#996600','#339933'],['#339933','#cc3333','#996600'],['#996600','#339933','#cc3333']];const p=phases[gst.tlPhase];gcx.fillStyle=p[0];gcx.beginPath();gcx.arc(x+3,y+3,2,0,Math.PI*2);gcx.fill();gcx.fillStyle=p[1];gcx.beginPath();gcx.arc(x+CS-3,y+3,2,0,Math.PI*2);gcx.fill();gcx.fillStyle=p[2];gcx.beginPath();gcx.arc(x+3,y+CS-3,2,0,Math.PI*2);gcx.fill();}
      }

      function gridDraw() {
        gcx.fillStyle='#080c18';gcx.fillRect(0,0,COLS*CS,ROWS*CS);
        const visitedSet=new Set,pathSet=new Set;
        if(gst.result){const{vis,path}=gst.result;const vc=Math.min(gst.step,vis.length);for(let i=0;i<vc;i++)visitedSet.add(vis[i]);const pc=Math.max(0,gst.step-vis.length);for(let i=0;i<pc;i++)pathSet.add(path[i]);}
        for(let r=0;r<ROWS;r++)for(let c=0;c<COLS;c++){const x=c*CS,y=r*CS;road(r,c)?gDrawRoad(r,c,x,y):gDrawBuilding(x,y);}
        if(gst.result){const vc2=GALGO_INFO[gst.result.algo].visited;for(const k of visitedSet){if(pathSet.has(k))continue;const[r,c]=GFK(k);gcx.fillStyle=vc2;gcx.fillRect(c*CS,r*CS,CS,CS);gcx.fillStyle=vc2.slice(0,7)+'66';gcx.fillRect(c*CS+CS/2-2,r*CS+CS/2-2,4,4);}}
        if(gst.result?.path.length){const{path}=gst.result;const pc=Math.max(0,gst.step-gst.result.vis.length);for(let i=0;i<pc;i++){const[r,c]=GFK(path[i]);gcx.fillStyle='#aa880055';gcx.fillRect(c*CS+1,r*CS+1,CS-2,CS-2);}if(pc>1){gcx.strokeStyle='#ffcc00aa';gcx.lineWidth=2;gcx.beginPath();const[r0,c0]=GFK(path[0]);gcx.moveTo(c0*CS+CS/2,r0*CS+CS/2);for(let i=1;i<pc;i++){const[r,c]=GFK(path[i]);gcx.lineTo(c*CS+CS/2,r*CS+CS/2);}gcx.stroke();}}
        for(const k of gst.obs){const[r,c]=GFK(k),x=c*CS,y=r*CS;gcx.fillStyle='#6e1010';gcx.fillRect(x,y,CS,CS);gcx.strokeStyle='#cc2222';gcx.lineWidth=1.5;gcx.beginPath();gcx.moveTo(x+3,y+3);gcx.lineTo(x+CS-3,y+CS-3);gcx.moveTo(x+CS-3,y+3);gcx.lineTo(x+3,y+CS-3);gcx.stroke();}
        {const[r,c]=gst.start,x=c*CS,y=r*CS;gcx.fillStyle='#1a4008';gcx.fillRect(x,y,CS,CS);gcx.fillStyle='#9cd44e';gcx.font='bold 11px monospace';gcx.textAlign='center';gcx.textBaseline='middle';gcx.fillText('S',x+CS/2,y+CS/2+1);}
        {const[r,c]=gst.end,x=c*CS,y=r*CS;gcx.fillStyle='#601010';gcx.fillRect(x,y,CS,CS);gcx.fillStyle='#f5c4b3';gcx.font='bold 11px monospace';gcx.textAlign='center';gcx.textBaseline='middle';gcx.fillText('E',x+CS/2,y+CS/2+1);}
        if(gst.result){const{vis,path}=gst.result;const pc=Math.max(0,gst.step-vis.length);const vi=Math.min(pc-1,path.length-1);if(vi>=0){const[r,c]=GFK(path[vi]);const x=c*CS+1,y=r*CS+2;gcx.fillStyle='#00dd66';gcx.fillRect(x,y,CS-2,CS-6);gcx.fillStyle='#009944';gcx.fillRect(x+1,y+3,CS-4,CS-10);gcx.fillStyle='#111';gcx.fillRect(x,y+CS-7,3,3);gcx.fillRect(x+CS-4,y+CS-7,3,3);gcx.fillStyle='#ffee88';gcx.fillRect(x+1,y,3,2);gcx.fillRect(x+CS-4,y,3,2);}}
      }
      window.gridDraw = gridDraw;

      let graf=null, glastT=0;
      function gAnimLoop(t){if(!gst.running)return;const interval=Math.max(6,105-gst.speed*2);if(t-glastT>interval){glastT=t;const total=(gst.result?.vis.length??0)+(gst.result?.path.length??0);if(gst.step<total){gst.step++;gridDraw();gUpdateStats();}else{gst.running=false;return;}}graf=requestAnimationFrame(gAnimLoop);}
      function gStartAnim(){gst.running=true;glastT=0;graf=requestAnimationFrame(gAnimLoop);}
      function gStopAnim(){gst.running=false;if(graf)cancelAnimationFrame(graf);}

      function gRun(){
        gStopAnim();const t0=performance.now();
        const res=GALGOS[gst.algo](gst.start,gst.end,gst.obs);
        res.algo=gst.algo;res.time=(performance.now()-t0).toFixed(2);
        gst.result=res;gst.step=0;gStartAnim();gUpdateStats();
      }
      function gUpdateStats(){
        if(!gst.result)return;
        const panel=document.getElementById('grid-result-panel');
        const title=document.getElementById('grid-result-title');
        const body=document.getElementById('grid-result-body');
        panel.style.display='block';const info=GALGO_INFO[gst.result.algo];
        title.style.color=info.color;title.textContent=`Result — ${info.name}`;
        const pl=gst.result.path.length,vc=Math.min(gst.step,gst.result.vis.length),pc=Math.max(0,gst.step-gst.result.vis.length);
        body.innerHTML=`<div>Nodes explored: <b style="color:var(--text)">${gst.result.vis.length}</b></div><div>Path length: <b style="color:var(--text)">${pl>0?pl+' cells':'No path found'}</b></div><div>Compute time: <b style="color:var(--text)">${gst.result.time} ms</b></div><div style="font-size:10px;color:var(--dim);margin-top:4px">${vc}/${gst.result.vis.length} explored · ${pc}/${pl} traversed</div>`;
      }

      function gCompare(){
        const results={};
        for(const[k,fn]of Object.entries(GALGOS)){const t0=performance.now();const r=fn(gst.start,gst.end,gst.obs);results[k]={...r,time:(performance.now()-t0).toFixed(2)};}
        const lens=Object.values(results).map(r=>r.path.length).filter(l=>l>0);
        const minL=lens.length?Math.min(...lens):Infinity;
        let html='<table style="width:100%;border-collapse:collapse;font-size:11.5px;font-family:JetBrains Mono,monospace"><thead><tr style="font-size:10px;color:var(--muted)"><th style="text-align:left;padding:6px;border-bottom:1px solid var(--border)">Algorithm</th><th style="text-align:right;padding:6px;border-bottom:1px solid var(--border)">Explored</th><th style="text-align:right;padding:6px;border-bottom:1px solid var(--border)">Path</th><th style="text-align:right;padding:6px;border-bottom:1px solid var(--border)">Time</th><th style="text-align:right;padding:6px;border-bottom:1px solid var(--border)">Optimal?</th></tr></thead><tbody>';
        for(const[k,r]of Object.entries(results)){const info=GALGO_INFO[k];const opt=r.path.length>0&&r.path.length===minL;html+=`<tr><td style="padding:6px;color:${info.color};font-weight:600">${info.name}</td><td style="text-align:right;padding:6px;color:var(--text)">${r.vis.length}</td><td style="text-align:right;padding:6px;color:var(--text)">${r.path.length||'—'}</td><td style="text-align:right;padding:6px;color:var(--text)">${r.time}</td><td style="text-align:right;padding:6px;color:${opt?'var(--dstar)':r.path.length?'var(--muted)':'#e05060'}">${r.path.length?(opt?'✓ Yes':'—'):'No path'}</td></tr>`;}
        html+='</tbody></table>';
        document.getElementById('grid-cmp-body').innerHTML=html;
        document.getElementById('grid-cmp-note').innerHTML=`<span style="color:${GALGO_INFO.astar.color}">A*</span> — heuristic-guided. &ensp;<span style="color:${GALGO_INFO.dijkstra.color}">Dijkstra</span> — uniform-cost. &ensp;<span style="color:${GALGO_INFO.bfs.color}">BFS</span> — layer-by-layer. &ensp;<span style="color:${GALGO_INFO.dstar.color}">D* Lite</span> — backward search.`;
        document.getElementById('grid-cmp-panel').style.display='block';
      }

      function gCellAt(e){const rect=gcv.getBoundingClientRect();const ex=e.touches?e.touches[0].clientX:e.clientX;const ey=e.touches?e.touches[0].clientY:e.clientY;const sx=gcv.width/rect.width,sy=gcv.height/rect.height;const c=Math.floor((ex-rect.left)*sx/CS);const r=Math.floor((ey-rect.top)*sy/CS);if(r<0||r>=ROWS||c<0||c>=COLS)return null;return[r,c];}
      function gInteract(r,c){if(!road(r,c))return;const k=GK(r,c),sk=GK(...gst.start),ek=GK(...gst.end);if(gst.mode==='start'){if(k===ek)return;gst.start=[r,c];gst.result=null;gst.step=0;document.getElementById('grid-result-panel').style.display='none';}else if(gst.mode==='end'){if(k===sk)return;gst.end=[r,c];gst.result=null;gst.step=0;document.getElementById('grid-result-panel').style.display='none';}else if(gst.mode==='obstacle'){if(k!==sk&&k!==ek)gst.obs.add(k);}else if(gst.mode==='erase'){gst.obs.delete(k);}gridDraw();}

      gcv.addEventListener('mousedown',e=>{gst.dragging=true;const c=gCellAt(e);if(c)gInteract(...c);});
      gcv.addEventListener('mousemove',e=>{if(!gst.dragging||(gst.mode!=='obstacle'&&gst.mode!=='erase'))return;const c=gCellAt(e);if(c)gInteract(...c);});
      gcv.addEventListener('mouseup',()=>gst.dragging=false);
      gcv.addEventListener('mouseleave',()=>gst.dragging=false);
      gcv.addEventListener('touchstart',e=>{e.preventDefault();const c=gCellAt(e);if(c)gInteract(...c);},{passive:false});
      gcv.addEventListener('touchmove',e=>{e.preventDefault();if(gst.mode==='obstacle'||gst.mode==='erase'){const c=gCellAt(e);if(c)gInteract(...c);}},{passive:false});

      // Build grid UI
      const gab=document.getElementById('grid-algo-btns');
      for(const[k,info]of Object.entries(GALGO_INFO)){
        const b=document.createElement('button');b.id='galgo_'+k;b.className='sbtn';b.style.setProperty('--clr',info.color);
        b.onclick=()=>{gst.algo=k;gStopAnim();gst.result=null;gst.step=0;document.getElementById('grid-result-panel').style.display='none';gRefreshAlgo();gridDraw();};
        gab.appendChild(b);
      }
      function gRefreshAlgo(){for(const[k,info]of Object.entries(GALGO_INFO)){const b=document.getElementById('galgo_'+k);b.className='sbtn'+(k===gst.algo?' active':'');b.style.setProperty('--clr',info.color);b.textContent=(k===gst.algo?'▶ ':'')+info.name;}}
      gRefreshAlgo();

      const gmb=document.getElementById('grid-mode-btns');
      const gmodes=[['start','Set Start Point'],['end','Set End Point'],['obstacle','Add Obstacle (drag)'],['erase','Erase Obstacle']];
      for(const[m,label]of gmodes){const b=document.createElement('button');b.id='gmode_'+m;b.className='sbtn';b.textContent=label;b.onclick=()=>{gst.mode=m;gRefreshModes();};gmb.appendChild(b);}
      function gRefreshModes(){gmodes.forEach(([m])=>{const b=document.getElementById('gmode_'+m);b.className='sbtn'+(m===gst.mode?' active':'');});}
      gRefreshModes();

      const gleg=document.getElementById('grid-legend');
      [['#131220','Road'],['#0e1525','Building'],['#1a4008','Start'],['#601010','End'],['#6e1010','Obstacle'],['#3a8ff044','Visited'],['#ffcc00','Path']].forEach(([col,lbl])=>{
        const d=document.createElement('div');d.className='grid-leg';
        d.innerHTML=`<div class="grid-leg-dot" style="background:${col};border:1px solid rgba(255,255,255,.1)"></div><span>${lbl}</span>`;
        gleg.appendChild(d);
      });

      document.getElementById('grid-speed').oninput=function(){gst.speed=+this.value;document.getElementById('grid-speed-val').textContent=this.value;};
      document.getElementById('grid-btn-run').onclick=gRun;
      document.getElementById('grid-btn-compare').onclick=gCompare;
      document.getElementById('grid-btn-clear').onclick=()=>{gStopAnim();gst.obs.clear();gst.result=null;gst.step=0;document.getElementById('grid-result-panel').style.display='none';document.getElementById('grid-cmp-panel').style.display='none';gridDraw();};
      document.getElementById('grid-cmp-close').onclick=()=>{document.getElementById('grid-cmp-panel').style.display='none';};
      document.getElementById('grid-btn-random').onclick=()=>{gStopAnim();gst.result=null;gst.step=0;document.getElementById('grid-result-panel').style.display='none';const sk=GK(...gst.start),ek=GK(...gst.end);let n=0;while(n<22){const r=Math.floor(Math.random()*ROWS),c=Math.floor(Math.random()*COLS);const k=GK(r,c);if(road(r,c)&&k!==sk&&k!==ek){gst.obs.add(k);n++;}}gridDraw();};

      setInterval(()=>{gst.tlPhase=(gst.tlPhase+1)%3;if(!gst.running)gridDraw();},1800);
      gridDraw();
    })();