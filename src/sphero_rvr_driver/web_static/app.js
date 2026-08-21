/* The console page. Vanilla on purpose: one SSE stream in, two POSTs out, and a
   canvas. All data is rendered via textContent — nothing from the wire is ever
   interpreted as HTML. Endings are rendered as what they are (batch A): the
   loop's own words, never softened. */
"use strict";

const $ = (id) => document.getElementById(id);
const chatEl = $("chat"), pillEl = $("statuspill"), dotEl = $("livedot");
const sendText = $("sendtext"), sendBtn = $("sendbtn"), sendBar = $("sendbar");
const canvas = $("map"), ctx = canvas.getContext("2d");
const poseBadge = $("posebadge"), mapMetaEl = $("mapmeta");

/* ------------------------------------------------------------------ chat */

const seen = new Set();          // event ids already rendered (replay + reconnect)

function entry(cls) {
  const div = document.createElement("div");
  div.className = "entry " + cls;
  return div;
}

function addText(cls, text) {
  const div = entry(cls);
  div.textContent = text;
  push(div);
}

function addCard(kind, title, text) {
  const div = entry("card " + kind);
  const t = document.createElement("div");
  t.className = "card-title";
  t.textContent = title;
  const body = document.createElement("div");
  body.textContent = text;
  div.append(t, body);
  push(div);
}

function addTool(ev) {
  const div = entry("tool");
  div.textContent = `tool ${ev.n}/${ev.max} — ${ev.call}`;
  push(div);
}

function addResult(ev) {
  if (ev.look) return addLook(ev.look);
  const div = entry("tool");
  const details = document.createElement("details");
  const summary = document.createElement("summary");
  const data = ev.data;
  let head = "result";
  if (data && typeof data.ok === "boolean") {
    head = (data.ok ? "ok" : "refused") + (data.message ? " — " + data.message : "");
  } else if (ev.text.length <= 90) {
    head = ev.text;
  }
  summary.textContent = head;
  const pre = document.createElement("pre");
  pre.textContent = data ? JSON.stringify(data, null, 1) : ev.text;
  details.append(summary, pre);
  div.append(details);
  push(div);
}

function chip(text, cls) {
  const span = document.createElement("span");
  span.className = "chip" + (cls ? " " + cls : "");
  span.textContent = text;
  return span;
}

function addLook(look) {
  const div = entry("look");
  if (look.photo) {
    const img = document.createElement("img");
    img.src = "/api/photo?name=" + encodeURIComponent(look.photo);
    img.alt = look.description || "look photo";
    img.onclick = () => window.open(img.src, "_blank");
    div.append(img);
  }
  const v = document.createElement("div");
  v.className = "verdicts";
  v.append(chip(look.match ? "match" : "no match", look.match ? "ok" : "bad"));
  if (look.identity) {
    v.append(chip(look.identity,
      { confirmed: "ok", mismatch: "bad", unverified: "warn" }[look.identity]));
  }
  if (typeof look.confidence === "number") v.append(chip("conf " + look.confidence));
  if (look.range_m != null) {
    v.append(chip(look.range_m.toFixed(2) + " m (" + (look.range_source || "?") + ")",
                  look.range_ambiguous ? "warn" : ""));
    if (look.range_ambiguous) v.append(chip("range ambiguous", "warn"));
  }
  if (look.bearing_deg != null) v.append(chip("bearing " + look.bearing_deg + "°"));
  if (look.bearing_relative_deg != null)
    v.append(chip("turn " + look.bearing_relative_deg + "°"));
  div.append(v);
  if (look.description) {
    const d = document.createElement("div");
    d.className = "desc";
    d.textContent = look.description;
    div.append(d);
  }
  const prov = document.createElement("div");
  prov.className = "prov";
  const pose = look.map_pose
    ? `(${look.map_pose.x}, ${look.map_pose.y}) @ ${look.map_pose.yaw_deg}°` : "?";
  prov.textContent =
    `target: ${look.target ?? "?"} · from ${pose} · ${look.stamp ?? ""} · ${look.model ?? ""}`;
  div.append(prov);
  push(div);
}

function push(div) {
  chatEl.append(div);
  chatEl.scrollTop = chatEl.scrollHeight;
}

function renderEvent(ev) {
  // state ticks are never replayed and arrive forever — keeping their ids
  // would grow the dedupe set without bound on a long-lived tab
  if (ev.type !== "state" && ev.id !== undefined) {
    if (seen.has(ev.id)) return;
    seen.add(ev.id);
  }
  switch (ev.type) {
    case "state": return onState(ev);
    case "instruction": return addText("you", ev.text);
    case "say": return addText("robot", ev.text);
    case "tool_call": return addTool(ev);
    case "tool_result": return addResult(ev);
    case "model_failure": return addCard("bad", "model failure", ev.text);
    case "refused": return addCard("bad", "contract refused", ev.text);
    case "budget": return addCard("warn", "budget exhausted", ev.text);
    case "reprompt": return addCard("warn", "reprompt", ev.text);
    case "error": return addCard("bad", "console error", ev.text);
    case "stop":
      return addCard("warn", "stop pressed",
        `goto: ${ev.goto_cancel ?? "?"} · mission: ${briefStop(ev.mission_stop)} · ${ev.note ?? ""}`);
    case "map_cleared": {
      const steps = ev.steps
        ? Object.entries(ev.steps).map(([k, v]) => `${k}: ${v}`).join(" · ") : "";
      return addCard(ev.ok ? "warn" : "bad",
        ev.ok ? "map cleared" : "map clear incomplete",
        (ev.message || "") + (steps ? " — " + steps : ""));
    }
    case "mission_end": return setBusy(false);
    case "note": return addText("sys", ev.text);
  }
}

function briefStop(msg) {
  try { const d = JSON.parse(msg); return d.message || msg; } catch { return msg ?? "?"; }
}

/* ------------------------------------------------------------------ state */

let busy = false;

function setBusy(b, progress) {
  busy = b;
  sendText.disabled = b;
  sendBtn.disabled = b;
  sendText.placeholder = b
    ? (progress ? `running — tool ${progress[0]}/${progress[1]}` : "running…")
    : "instruction…";
}

function onState(ev) {
  const chat = ev.chat || {};
  setBusy(chat.state === "running", chat.tool);
  const m = ev.mission || {};
  let text, cls = "";
  if (!m.available && m.stale) { text = `status stale ${m.age_s}s`; cls = "warn"; }
  else if (!m.available) { text = "explorer quiet"; }
  else {
    const d = m.data || {};
    if (d.done) text = "mission finished";
    else if (d.running || d.armed) { text = "exploring"; cls = "running"; }
    else text = "idle";
    if (chat.state === "running") { text = "instruction running"; cls = "running"; }
  }
  pillEl.textContent = text;
  pillEl.className = "pill " + cls;
  poseBadge.classList.toggle("hidden", !!ev.pose);
  if (ev.pose) targetPose = ev.pose;
  if (ev.map) {
    if (!mapMeta || ev.map.stamp !== mapMeta.stamp) fetchMap(ev.map);
    mapMeta = ev.map;
    mapMetaEl.textContent =
      `${(ev.map.width * ev.map.resolution_m).toFixed(1)}×` +
      `${(ev.map.height * ev.map.resolution_m).toFixed(1)} m · ` +
      `${ev.map.known_pct}% known`;
  }
}

/* ------------------------------------------------------------------ SSE */

let source = null;

function connect() {
  source = new EventSource("/api/events");
  source.onopen = () => dotEl.classList.remove("off");
  source.onerror = () => {           // EventSource reconnects itself (Last-Event-ID)
    dotEl.classList.add("off");
    pillEl.textContent = "reconnecting…";
    pillEl.className = "pill warn";
  };
  source.onmessage = (msg) => {
    try { renderEvent(JSON.parse(msg.data)); } catch (e) { /* keep the stream */ }
  };
}

/* ------------------------------------------------------------------ send */

sendBar.addEventListener("submit", async (e) => {
  e.preventDefault();
  const text = sendText.value.trim();
  if (!text || busy) return;
  sendText.value = "";
  try {
    const resp = await fetch("/api/instruction", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text }),
    });
    if (!resp.ok) {
      const body = await resp.json().catch(() => ({}));
      addCard(resp.status === 409 ? "warn" : "bad", "not sent",
              body.error || `HTTP ${resp.status}`);
    } else {
      setBusy(true);
    }
  } catch (err) {
    addCard("bad", "not sent", "console unreachable: " + err.message);
  }
});

/* new map: destructive-ish, so one gentle two-tap confirm (no dialog) */
const newMapBtn = $("newmapbtn");
let confirmTimer = null;
newMapBtn.addEventListener("click", async () => {
  if (!newMapBtn.classList.contains("confirm")) {
    newMapBtn.classList.add("confirm");
    newMapBtn.textContent = "tap again to forget map";
    confirmTimer = setTimeout(() => {
      newMapBtn.classList.remove("confirm");
      newMapBtn.textContent = "new map";
    }, 4000);
    return;
  }
  clearTimeout(confirmTimer);
  newMapBtn.classList.remove("confirm");
  newMapBtn.textContent = "clearing…";
  newMapBtn.disabled = true;
  try {
    await fetch("/api/map/clear", { method: "POST" });
    // the result card arrives through the stream (type "map_cleared")
  } catch (err) {
    addCard("bad", "map clear failed", "console unreachable: " + err.message);
  }
  newMapBtn.textContent = "new map";
  newMapBtn.disabled = false;
});

$("stopbtn").addEventListener("click", async () => {
  document.body.classList.remove("stopping");
  void document.body.offsetWidth;          // restart the pulse animation
  document.body.classList.add("stopping");
  try {
    await fetch("/api/stop", { method: "POST" });
    // the result card arrives through the stream (type "stop")
  } catch (err) {
    addCard("bad", "stop request failed", err.message);
  }
});

/* ------------------------------------------------------------------ map */

let mapMeta = null;                  // latest meta from the state tick
let mapImage = null;                 // decoded PNG
let targetPose = null;               // latest pose from the tick
let shownPose = null;                // eased pose actually drawn
let view = { scale: 1, x: 0, y: 0, fitted: false };

function fetchMap(meta) {
  const img = new Image();
  img.onload = () => { mapImage = img; view.fitted = false; };
  img.src = "/api/map.png?stamp=" + encodeURIComponent(meta.stamp);
}

function fitView() {
  if (!mapImage) return;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const scale = 0.92 * Math.min(w / mapImage.width, h / mapImage.height);
  view = { scale,
           x: (w - mapImage.width * scale) / 2,
           y: (h - mapImage.height * scale) / 2,
           fitted: true };
}

function worldToPng(wx, wy) {
  const m = mapMeta;
  return [(wx - m.origin.x) / m.resolution_m,
          m.height - (wy - m.origin.y) / m.resolution_m];
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    view.fitted = false;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (mapImage && mapMeta) {
    if (!view.fitted) fitView();
    ctx.imageSmoothingEnabled = view.scale < 3;
    ctx.setTransform(dpr * view.scale, 0, 0, dpr * view.scale,
                     dpr * view.x, dpr * view.y);
    ctx.drawImage(mapImage, 0, 0);
    if (targetPose) {
      // ease toward the tick's pose so motion reads as motion, not teleports
      if (!shownPose) shownPose = { ...targetPose };
      shownPose.x += (targetPose.x - shownPose.x) * 0.18;
      shownPose.y += (targetPose.y - shownPose.y) * 0.18;
      let dy = targetPose.yaw_deg - shownPose.yaw_deg;
      while (dy > 180) dy -= 360;
      while (dy < -180) dy += 360;
      shownPose.yaw_deg += dy * 0.18;
      const [px, py] = worldToPng(shownPose.x, shownPose.y);
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-shownPose.yaw_deg * Math.PI / 180);   // canvas y points down
      const s = Math.max(9 / view.scale, 3);            // constant on-screen size
      ctx.beginPath();
      ctx.moveTo(1.4 * s, 0);
      ctx.lineTo(-0.8 * s, 0.8 * s);
      ctx.lineTo(-0.4 * s, 0);
      ctx.lineTo(-0.8 * s, -0.8 * s);
      ctx.closePath();
      ctx.fillStyle = "#ff6a2b";
      ctx.strokeStyle = "rgba(0,0,0,0.55)";
      ctx.lineWidth = 1 / view.scale;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }
  }
  requestAnimationFrame(draw);
}

/* pan / pinch / wheel: the canvas owns its gestures (touch-action: none) */
const pointers = new Map();
let pinchStart = null;

canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
  if (pointers.size === 2) {
    const [a, b] = [...pointers.values()];
    pinchStart = { dist: Math.hypot(a.x - b.x, a.y - b.y), scale: view.scale };
  }
});
canvas.addEventListener("pointermove", (e) => {
  const prev = pointers.get(e.pointerId);
  if (!prev) return;
  const cur = { x: e.clientX, y: e.clientY };
  if (pointers.size === 1) {
    view.x += cur.x - prev.x;
    view.y += cur.y - prev.y;
  } else if (pointers.size === 2 && pinchStart) {
    pointers.set(e.pointerId, cur);
    const [a, b] = [...pointers.values()];
    const dist = Math.hypot(a.x - b.x, a.y - b.y);
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    zoomAt(mid.x, mid.y, pinchStart.scale * (dist / pinchStart.dist) / view.scale);
  }
  pointers.set(e.pointerId, cur);
});
function endPointer(e) {
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinchStart = null;
}
canvas.addEventListener("pointerup", endPointer);
canvas.addEventListener("pointercancel", endPointer);
canvas.addEventListener("dblclick", () => { view.fitted = false; });
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  zoomAt(e.offsetX, e.offsetY, Math.exp(-e.deltaY * 0.0015));
}, { passive: false });

function zoomAt(cx, cy, factor) {
  const next = Math.min(Math.max(view.scale * factor, 0.2), 40);
  factor = next / view.scale;
  view.x = cx - (cx - view.x) * factor;
  view.y = cy - (cy - view.y) * factor;
  view.scale = next;
}

/* ------------------------------------------------------------------ go */

connect();
requestAnimationFrame(draw);
