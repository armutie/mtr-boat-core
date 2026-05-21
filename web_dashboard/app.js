const state = {
  last: null,
  throttle: [],
  accel: [],
  gnss: [],
  maxTrace: 190,
  maxTrack: 240,
  control: {
    mode: "off",
    intent: { throttle: 0, steering: 0 },
    wheelAngleDeg: 0,
    wheelMaxDeg: 115,
    springReturn: true,
    activePointer: null,
    lastPointerAngleDeg: 0,
    activeThrottlePointer: null,
    lastSentAt: 0,
    serverReachable: true,
    sse: { everConnected: false, lastEventAt: 0 },
    stopRetry: null,
    serverManual: null,
    limits: { manual_slew_per_s: 0.0, stale_after_s: 0.45 },
    actuatorErrorAt: 0,
    pendingMode: null,
    pendingModeExpires: 0,
  },
};

const colors = {
  bg: "#fffdf4",
  panel: "#fff7df",
  grid: "#ead8ae",
  line: "#d3b982",
  ink: "#2c2230",
  muted: "#786a72",
  teal: "#2a9d8f",
  green: "#5b9f55",
  yellow: "#f2b84b",
  red: "#d94f5c",
  orange: "#df7f3f",
  blue: "#547aa5",
  dead: "#a89678",
};

const views = document.querySelectorAll(".view");
const tabs = document.querySelectorAll(".tab");

tabs.forEach((tab) => {
  tab.addEventListener("click", () => {
    tabs.forEach((item) => item.classList.remove("active"));
    views.forEach((item) => item.classList.remove("active"));
    tab.classList.add("active");
    document.getElementById(tab.dataset.tab)?.classList.add("active");
    requestAnimationFrame(() => drawAll(state.last));
  });
});

const wheelCanvas = document.getElementById("manual-wheel");
const throttleCanvas = document.getElementById("manual-throttle");
const stopButton = document.getElementById("manual-stop");
const springToggle = document.getElementById("manual-spring-toggle");
const modePills = Array.from(document.querySelectorAll(".mode-pill"));
const controlSurface = document.querySelector(".control-surface");

// Cache the high-traffic readout nodes so per-pointermove updates don't pay
// for a querySelector on every event.
const readoutNodes = {
  steerValue: document.getElementById("manual-steering-value"),
  throttleValue: document.getElementById("manual-throttle-value"),
  throttleInput: document.getElementById("manual-throttle-input"),
  leftIntent: document.getElementById("manual-left-intent"),
  rightIntent: document.getElementById("manual-right-intent"),
  steerBarLeft: document.querySelector(".steer-bar .bar-left"),
  steerBarRight: document.querySelector(".steer-bar .bar-right"),
  throttleBarUp: document.querySelector(".throttle-bar .bar-up"),
};

// Coalesce intent-driven redraws + DOM updates so that even at 300+ Hz
// pointermove the work runs at most once per animation frame (~60 Hz).
// Without this, heavy pointermove handlers starve setInterval and SSE
// callbacks, causing the visible "Applied catches up after release" bug.
let localRenderQueued = false;
function scheduleLocalRender() {
  if (localRenderQueued) return;
  localRenderQueued = true;
  requestAnimationFrame(() => {
    localRenderQueued = false;
    drawWheel();
    drawThrottle();
    renderLocalManualIntent();
  });
}

function isControlEnabled() {
  return state.control.mode === "manual" && state.control.serverReachable;
}

function bindManualControls() {
  if (wheelCanvas) {
    wheelCanvas.addEventListener("pointerdown", onWheelPointerDown);
    wheelCanvas.addEventListener("pointermove", onWheelPointerMove);
    wheelCanvas.addEventListener("pointerup", onWheelPointerEnd);
    wheelCanvas.addEventListener("pointercancel", onWheelPointerEnd);
    wheelCanvas.addEventListener("lostpointercapture", onWheelPointerEnd);
    wheelCanvas.addEventListener("keydown", onWheelKeyDown);
  }
  if (throttleCanvas) {
    throttleCanvas.addEventListener("pointerdown", onThrottlePointerDown);
    throttleCanvas.addEventListener("pointermove", onThrottlePointerMove);
    throttleCanvas.addEventListener("pointerup", onThrottlePointerEnd);
    throttleCanvas.addEventListener("pointercancel", onThrottlePointerEnd);
    throttleCanvas.addEventListener("lostpointercapture", onThrottlePointerEnd);
    throttleCanvas.addEventListener("keydown", onThrottleKeyDown);
    throttleCanvas.addEventListener("wheel", onThrottleWheel, { passive: false });
  }
  stopButton?.addEventListener("click", () => stopManualCommand());
  springToggle?.addEventListener("click", () => {
    state.control.springReturn = !state.control.springReturn;
    springToggle.setAttribute("aria-pressed", state.control.springReturn ? "true" : "false");
    springToggle.textContent = `Spring return: ${state.control.springReturn ? "on" : "off"}`;
  });
  modePills.forEach((pill) => {
    pill.addEventListener("click", () => requestModeChange(pill.dataset.mode));
  });
  document.addEventListener("keydown", onGlobalKeyDown);
  renderLocalManualIntent();
  drawWheel();
  drawThrottle();
}

function pointerAngleDeg(canvas, evt) {
  const rect = canvas.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  return (Math.atan2(evt.clientY - cy, evt.clientX - cx) * 180) / Math.PI;
}

function onWheelPointerDown(evt) {
  if (!isControlEnabled()) return;
  evt.preventDefault();
  wheelCanvas.setPointerCapture(evt.pointerId);
  state.control.activePointer = evt.pointerId;
  state.control.lastPointerAngleDeg = pointerAngleDeg(wheelCanvas, evt);
}

function onWheelPointerMove(evt) {
  if (state.control.activePointer !== evt.pointerId) return;
  evt.preventDefault();
  const angle = pointerAngleDeg(wheelCanvas, evt);
  // Incremental rotation: delta is computed against the previous frame, not
  // the original grab angle. This means each step is a small angular nudge
  // even when the cursor crosses the ±180° branch behind the wheel, which
  // prevents the "drag through the bottom of the wheel and snap to the
  // opposite lock" behaviour. Once the wheel is at ±wheelMaxDeg the clamp
  // holds; to come back you have to drag the cursor the other way.
  let delta = angle - state.control.lastPointerAngleDeg;
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;
  state.control.lastPointerAngleDeg = angle;
  const target = state.control.wheelAngleDeg + delta;
  setWheelAngle(target);
}

function onWheelPointerEnd(evt) {
  if (state.control.activePointer !== evt.pointerId) return;
  state.control.activePointer = null;
  try {
    wheelCanvas.releasePointerCapture(evt.pointerId);
  } catch (_) {}
  if (state.control.springReturn) {
    animateWheelTo(0);
  }
}

function onWheelKeyDown(evt) {
  if (!isControlEnabled()) return;
  const step = evt.shiftKey ? 12 : 4;
  if (evt.key === "ArrowLeft") {
    evt.preventDefault();
    setWheelAngle(state.control.wheelAngleDeg - step);
  } else if (evt.key === "ArrowRight") {
    evt.preventDefault();
    setWheelAngle(state.control.wheelAngleDeg + step);
  } else if (evt.key === "Home" || evt.key === "0") {
    evt.preventDefault();
    setWheelAngle(0);
  }
}

function setWheelAngle(angleDeg) {
  state.control.wheelAngleDeg = clamp(angleDeg, -state.control.wheelMaxDeg, state.control.wheelMaxDeg);
  state.control.intent.steering = state.control.wheelAngleDeg / state.control.wheelMaxDeg;
  wheelCanvas?.setAttribute("aria-valuenow", state.control.intent.steering.toFixed(2));
  scheduleLocalRender();
}

function animateWheelTo(target) {
  const start = state.control.wheelAngleDeg;
  const t0 = performance.now();
  const duration = 220;
  function step(now) {
    if (state.control.activePointer !== null) return;
    const t = Math.min(1, (now - t0) / duration);
    const ease = 1 - Math.pow(1 - t, 3);
    setWheelAngle(start + (target - start) * ease);
    if (t < 1) requestAnimationFrame(step);
  }
  requestAnimationFrame(step);
}

function throttleFromPointer(canvas, evt) {
  const rect = canvas.getBoundingClientRect();
  const padding = 14;
  const trackTop = rect.top + padding;
  const trackBottom = rect.bottom - padding;
  const y = evt.clientY;
  const clamped = Math.max(trackTop, Math.min(trackBottom, y));
  return clamp(1 - (clamped - trackTop) / (trackBottom - trackTop), 0, 1);
}

function onThrottlePointerDown(evt) {
  if (!isControlEnabled()) return;
  evt.preventDefault();
  throttleCanvas.setPointerCapture(evt.pointerId);
  state.control.activeThrottlePointer = evt.pointerId;
  setThrottleIntent(throttleFromPointer(throttleCanvas, evt));
}

function onThrottlePointerMove(evt) {
  if (state.control.activeThrottlePointer !== evt.pointerId) return;
  evt.preventDefault();
  setThrottleIntent(throttleFromPointer(throttleCanvas, evt));
}

function onThrottlePointerEnd(evt) {
  if (state.control.activeThrottlePointer !== evt.pointerId) return;
  state.control.activeThrottlePointer = null;
  try {
    throttleCanvas.releasePointerCapture(evt.pointerId);
  } catch (_) {}
}

function onThrottleKeyDown(evt) {
  if (!isControlEnabled()) return;
  const step = evt.shiftKey ? 0.1 : 0.02;
  if (evt.key === "ArrowUp") {
    evt.preventDefault();
    setThrottleIntent(state.control.intent.throttle + step);
  } else if (evt.key === "ArrowDown") {
    evt.preventDefault();
    setThrottleIntent(state.control.intent.throttle - step);
  } else if (evt.key === "Home" || evt.key === "End" || evt.key === "0") {
    evt.preventDefault();
    setThrottleIntent(0);
  }
}

function onThrottleWheel(evt) {
  if (!isControlEnabled()) return;
  evt.preventDefault();
  const dir = evt.deltaY < 0 ? 1 : -1;
  setThrottleIntent(state.control.intent.throttle + dir * 0.04);
}

function setThrottleIntent(value) {
  state.control.intent.throttle = clamp(value, 0, 1);
  throttleCanvas?.setAttribute("aria-valuenow", state.control.intent.throttle.toFixed(2));
  scheduleLocalRender();
}

function onGlobalKeyDown(evt) {
  const target = evt.target;
  const tag = target?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || target?.isContentEditable) return;
  if (evt.key === " " || evt.key === "Spacebar" || evt.key === "Escape") {
    evt.preventDefault();
    stopManualCommand();
  }
}

function requestModeChange(mode) {
  if (!mode || mode === state.control.mode) return;
  if (mode !== "manual") {
    resetIntent();
  }
  state.control.mode = mode;
  state.control.pendingMode = mode;
  state.control.pendingModeExpires = performance.now() + 1500;
  syncModePills();
  renderLocalManualIntent();
  postJson("/api/control/mode", { mode })
    .then(() => {
      state.control.serverReachable = true;
    })
    .catch((error) => {
      state.control.serverReachable = false;
      state.control.pendingMode = null;
      setReason(`mode change failed: ${error.message}`, "error");
    });
}

function syncModePills() {
  modePills.forEach((pill) => {
    const active = pill.dataset.mode === state.control.mode;
    pill.setAttribute("aria-checked", active ? "true" : "false");
  });
  controlSurface?.setAttribute("data-mode", state.control.mode);
  const banner = $("#manual-banner");
  if (banner) {
    if (state.control.mode === "auto") {
      banner.hidden = false;
      banner.dataset.tone = "warn";
      banner.textContent = "Auto mode is a stub. Waypoints not yet implemented; output stays neutral.";
    } else {
      banner.hidden = true;
      banner.removeAttribute("data-tone");
      banner.textContent = "";
    }
  }
}

function resetIntent() {
  state.control.intent.throttle = 0;
  state.control.intent.steering = 0;
  state.control.wheelAngleDeg = 0;
  drawWheel();
  drawThrottle();
}

function setReason(message, tone) {
  text("#manual-reason", message);
  const reasonEl = $("#manual-reason");
  if (!reasonEl) return;
  if (tone) reasonEl.dataset.tone = tone;
  else reasonEl.removeAttribute("data-tone");
}

function $(selector) {
  return document.querySelector(selector);
}

function text(selector, value) {
  const node = $(selector);
  if (node) node.textContent = value;
}

function healthOf(data, sensor) {
  return data?.health?.[sensor] || "unavailable";
}

function setHealth(selector, health) {
  const node = $(selector);
  if (!node) return;
  node.dataset.health = health;
  node.textContent = health;
}

function setPanelHealth(selector, health) {
  const node = $(selector);
  if (node) node.dataset.health = health;
}

function num(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function fmt(value, digits = 2, suffix = "") {
  const n = num(value);
  if (n === null) return "--";
  return `${n.toFixed(digits)}${suffix}`;
}

function fmtInt(value) {
  const n = num(value);
  if (n === null) return "--";
  return `${Math.round(n)}`;
}

function fmtAge(value) {
  const n = num(value);
  return n === null ? "--" : `${n.toFixed(1)}s`;
}

function clamp(value, min = 0, max = 1) {
  return Math.max(min, Math.min(max, value));
}

function pushBounded(list, value, maxLength) {
  list.push(value);
  if (list.length > maxLength) list.splice(0, list.length - maxLength);
}

function getPoints(mmwave) {
  return mmwave?.points || mmwave?.filtered_points || [];
}

function pointSnr(point) {
  return num(point?.snr) ?? num(point?.snr_raw) ?? num(point?.mean_snr_raw);
}

function pointDoppler(point) {
  return num(point?.doppler) ?? num(point?.velocity) ?? num(point?.mean_doppler);
}

function average(values) {
  const nums = values.filter((value) => value !== null);
  if (!nums.length) return null;
  return nums.reduce((sum, value) => sum + value, 0) / nums.length;
}

function signed(value) {
  const n = num(value) ?? 0;
  return `${n >= 0 ? "+" : ""}${n.toFixed(2)}`;
}

function manualMix(throttle, steering) {
  const t = clamp(throttle, 0, 1);
  const s = clamp(steering, -1, 1);
  return {
    left: clamp(t * (1 + s), 0, 1),
    right: clamp(t * (1 - s), 0, 1),
  };
}

function renderLocalManualIntent() {
  const intent = state.control.intent;
  const mix = manualMix(intent.throttle, intent.steering);
  if (readoutNodes.steerValue) readoutNodes.steerValue.textContent = signed(intent.steering);
  if (readoutNodes.throttleValue) readoutNodes.throttleValue.textContent = fmt(intent.throttle, 2);
  if (readoutNodes.throttleInput) readoutNodes.throttleInput.textContent = fmt(intent.throttle, 2);
  if (readoutNodes.leftIntent) readoutNodes.leftIntent.textContent = fmt(mix.left, 2);
  if (readoutNodes.rightIntent) readoutNodes.rightIntent.textContent = fmt(mix.right, 2);
  setSteerBar(intent.steering);
  setThrottleBar(intent.throttle);
}

function setSteerBar(value) {
  const v = clamp(num(value) ?? 0, -1, 1);
  const left = v < 0 ? -v : 0;
  const right = v > 0 ? v : 0;
  readoutNodes.steerBarLeft?.style.setProperty("--fill", left.toFixed(3));
  readoutNodes.steerBarRight?.style.setProperty("--fill", right.toFixed(3));
}

function setThrottleBar(value) {
  const v = clamp(num(value) ?? 0, 0, 1);
  readoutNodes.throttleBarUp?.style.setProperty("--fill", v.toFixed(3));
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    let detail = `${response.status} ${response.statusText}`;
    try {
      const body = await response.json();
      if (body?.error) detail = body.error;
    } catch (_) {}
    throw new Error(detail);
  }
  return response.json();
}

function sendManualCommand(force = false) {
  if (state.control.mode !== "manual") return;
  const now = performance.now();
  if (!force && now - state.control.lastSentAt < 45) return;
  state.control.lastSentAt = now;
  postJson("/api/control/manual", {
    enabled: true,
    throttle: state.control.intent.throttle,
    steering: state.control.intent.steering,
  })
    .then(() => {
      state.control.serverReachable = true;
    })
    .catch((error) => {
      state.control.serverReachable = false;
      setReason(`server command failed: ${error.message}`, "error");
    });
}

function stopManualCommand() {
  state.control.mode = "off";
  state.control.pendingMode = "off";
  state.control.pendingModeExpires = performance.now() + 2000;
  resetIntent();
  syncModePills();
  renderLocalManualIntent();
  if (stopButton) {
    stopButton.dataset.pressed = "true";
    setTimeout(() => stopButton.removeAttribute("data-pressed"), 700);
  }
  startStopRetry();
}

function startStopRetry() {
  const retry = { startedAt: performance.now(), attempts: 0, acked: false };
  state.control.stopRetry = retry;
  function attempt() {
    if (retry.acked) return;
    if (state.control.stopRetry !== retry) return;
    retry.attempts += 1;
    postJson("/api/control/stop")
      .then(() => {
        retry.acked = true;
        state.control.stopRetry = null;
        state.control.serverReachable = true;
        setReason("operator stop acknowledged", null);
      })
      .catch((error) => {
        state.control.serverReachable = false;
        const elapsed = performance.now() - retry.startedAt;
        if (elapsed >= 1500) {
          state.control.stopRetry = null;
          setReason(`stop NOT acknowledged after retries: ${error.message}`, "error");
          return;
        }
        setReason(`local stop only; retrying (${retry.attempts})...`, "warn");
        setTimeout(attempt, 100);
      });
  }
  attempt();
}

function updateHistory(data) {
  const throttle = num(data?.mmwave?.control?.throttle);
  const accel = num(data?.imu?.accel_mag_g);
  const lat = num(data?.gnss?.lat);
  const lon = num(data?.gnss?.lon);
  const mmHealth = healthOf(data, "mmwave");
  const gnssHealth = healthOf(data, "gnss");
  const imuHealth = healthOf(data, "imu");

  if (mmHealth === "live" && throttle !== null) pushBounded(state.throttle, throttle, state.maxTrace);
  if (imuHealth === "live" && accel !== null) pushBounded(state.accel, accel, state.maxTrace);
  if (gnssHealth === "live" && lat !== null && lon !== null && data?.gnss?.fix !== "none") {
    const last = state.gnss[state.gnss.length - 1];
    if (!last || Math.abs(last.lat - lat) > 0.0000001 || Math.abs(last.lon - lon) > 0.0000001) {
      pushBounded(state.gnss, { lat, lon, heading: num(data?.gnss?.heading_deg) }, state.maxTrack);
    }
  }
}

function updateDom(data) {
  const mmHealth = healthOf(data, "mmwave");
  const gnssHealth = healthOf(data, "gnss");
  const imuHealth = healthOf(data, "imu");
  const mmwave = data.mmwave || {};
  const gnss = data.gnss || {};
  const imu = data.imu || {};
  const manual = data.manual_control || {};
  const manualOutput = manual.output || {};
  const manualInput = manual.input || {};
  const control = mmwave.control || {};
  const counts = mmwave.point_count || {};
  const raw = mmwave.raw_points || [];
  const filtered = getPoints(mmwave);
  const clusters = mmwave.clusters || [];
  const scores = mmwave.scores || mmwave.zones || {};
  const rejected = Math.max(0, (counts.raw ?? raw.length) - (counts.filtered ?? filtered.length));
  const filterRatio = raw.length ? filtered.length / raw.length : null;
  const singletons = clusters.filter((cluster) => cluster.is_singleton).length;
  const allSnr = raw.map(pointSnr);
  const allDoppler = raw.map(pointDoppler);
  const avgSnr = average(allSnr);
  const avgDoppler = average(allDoppler);

  $("#stream-dot")?.parentElement?.setAttribute("data-health", mmHealth === "live" || gnssHealth === "live" || imuHealth === "live" ? "live" : mmHealth);
  text("#stream-label", `${data.mode || "waiting"} stream`);
  text("#stream-meta", `up ${fmt(data.session?.uptime_s, 0, "s")}`);

  setHealth("#overview-mmwave-health", mmHealth);
  setHealth("#overview-gnss-health", gnssHealth);
  setHealth("#overview-imu-health", imuHealth);
  setHealth("#radar-health", mmHealth);
  setHealth("#gnss-health", gnssHealth);
  setHealth("#imu-health", imuHealth);
  setPanelHealth(".radar-primary", mmHealth);
  setPanelHealth(".mini-map", gnssHealth);
  setPanelHealth(".attitude-card", imuHealth);
  setPanelHealth("#radar .detail-main", mmHealth);
  setPanelHealth("#gnss .detail-main", gnssHealth);
  setPanelHealth("#imu .detail-main", imuHealth);
  setPanelHealth(".compass-panel", gnssHealth);

  text("#overview-command", (mmwave.command || "unavailable").replaceAll("_", " "));
  text("#overview-reason", mmwave.reason || "No mmWave feed connected.");
  text("#overview-speed", fmt(gnss.speed_mps, 2, " m/s"));
  text("#overview-heading", fmt(gnss.heading_deg, 0, " deg"));
  text("#overview-fix", gnss.fix || "--");
  text("#overview-sats", fmtInt(gnss.satellites));
  text("#overview-throttle", mmHealth === "live" ? `${fmt(control.throttle, 2)} / ${fmt(control.target_throttle, 2)}` : "-- / --");
  text("#overview-accel", fmt(imu.accel_mag_g, 3, " g"));
  setZone("#overview-zone-left", scores.left);
  setZone("#overview-zone-front", scores.front);
  setZone("#overview-zone-right", scores.right);

  text("#radar-command", (mmwave.command || "unavailable").replaceAll("_", " "));
  text("#radar-age", fmtAge(mmwave.age_s));
  text("#radar-frame", mmwave.frame_number ?? "--");
  text("#radar-raw", counts.raw ?? raw.length);
  text("#radar-filtered", counts.filtered ?? filtered.length);
  text("#radar-clusters", counts.clusters ?? clusters.length);
  text("#radar-rejected", rejected);
  text("#radar-filter-ratio", filterRatio === null ? "--" : `${Math.round(filterRatio * 100)}%`);
  text("#radar-singletons", singletons);
  text("#radar-snr", fmt(avgSnr, 0));
  text("#radar-doppler", fmt(avgDoppler, 2, " m/s"));
  text("#radar-throttle", fmt(control.throttle, 2));
  text("#radar-target-throttle", fmt(control.target_throttle, 2));
  text("#radar-steering", fmt(control.steering, 2));
  text("#radar-target-steering", fmt(control.target_steering, 2));
  text("#radar-reason", mmwave.reason || "No mmWave feed connected.");
  text("#radar-metadata", JSON.stringify(mmwave.metadata || {}, null, 2));
  setBar("#radar-throttle-bar", mmHealth === "live" ? control.throttle : null, 0, 1, "var(--green)");
  setBar("#radar-target-throttle-bar", mmHealth === "live" ? control.target_throttle : null, 0, 1, "var(--cyan)");
  setBar("#radar-steering-bar", mmHealth === "live" ? Math.abs(num(control.steering) ?? 0) : null, 0, 1, "var(--yellow)");
  setBar("#radar-target-steering-bar", mmHealth === "live" ? Math.abs(num(control.target_steering) ?? 0) : null, 0, 1, "var(--orange)");

  text("#gnss-lat", fmt(gnss.lat, 7));
  text("#gnss-lon", fmt(gnss.lon, 7));
  text("#gnss-speed", fmt(gnss.speed_mps, 2, " m/s"));
  text("#gnss-heading", fmt(gnss.heading_deg, 0, " deg"));
  text("#gnss-fix", gnss.fix || "--");
  text("#gnss-sats", fmtInt(gnss.satellites));
  text("#gnss-hdop", fmt(gnss.hdop, 2));
  text("#gnss-age", fmtAge(gnss.age_s));
  text("#gnss-source", `source ${gnss.source || "--"}`);
  text("#session-logging", data.session?.logging ? `logging ${Object.values(data.session.log_paths || {}).join(" ")}` : "logging off");

  text("#imu-ax", fmt(imu.accel_x_g, 3, " g"));
  text("#imu-ay", fmt(imu.accel_y_g, 3, " g"));
  text("#imu-az", fmt(imu.accel_z_g, 3, " g"));
  text("#imu-amag", fmt(imu.accel_mag_g, 3, " g"));
  text("#imu-gx", fmt(imu.gyro_x_dps, 1));
  text("#imu-gy", fmt(imu.gyro_y_dps, 1));
  text("#imu-gz", fmt(imu.gyro_z_dps, 1));
  text("#imu-yaw", fmt(imu.yaw_relative_deg, 1, " deg"));
  text("#imu-dt", fmt(imu.dt_s, 3, "s"));
  text("#imu-age", fmtAge(imu.age_s));
  text("#imu-source", imu.source || "--");
  const bias = imu.bias || {};
  text("#imu-bias", imu.error ? `error ${imu.error}` : `bias x ${fmt(bias.x_dps, 2)} y ${fmt(bias.y_dps, 2)} z ${fmt(bias.z_dps, 2)}`);
  setBar("#imu-gx-bar", imuHealth === "live" ? Math.abs(num(imu.gyro_x_dps) ?? 0) : null, 0, 120, "var(--cyan)");
  setBar("#imu-gy-bar", imuHealth === "live" ? Math.abs(num(imu.gyro_y_dps) ?? 0) : null, 0, 120, "var(--yellow)");
  setBar("#imu-gz-bar", imuHealth === "live" ? Math.abs(num(imu.gyro_z_dps) ?? 0) : null, 0, 120, "var(--orange)");

  state.control.serverManual = manual;
  if (manual.limits) {
    state.control.limits = {
      manual_slew_per_s: num(manual.limits.manual_slew_per_s) ?? state.control.limits.manual_slew_per_s,
      stale_after_s: num(manual.limits.stale_after_s) ?? state.control.limits.stale_after_s,
    };
  }
  if (manual.mode && state.control.pendingMode) {
    if (manual.mode === state.control.pendingMode) {
      state.control.pendingMode = null;
    } else if (performance.now() > state.control.pendingModeExpires) {
      state.control.pendingMode = null;
    }
  }
  if (
    manual.mode &&
    state.control.pendingMode === null &&
    state.control.activePointer === null &&
    state.control.activeThrottlePointer === null &&
    state.control.stopRetry === null
  ) {
    if (state.control.mode !== manual.mode) {
      state.control.mode = manual.mode;
      syncModePills();
    }
  }
  const readoutMode = state.control.pendingMode || manual.mode || state.control.mode || "off";
  text("#manual-mode", readoutMode);
  text("#manual-actuator", typeof manual.actuator === "string" ? manual.actuator : (manual.actuator?.actuator || "dry-run"));
  const effective = manual.effective || {};
  text("#manual-pwm-left", fmtInt(effective.left_us));
  text("#manual-pwm-right", fmtInt(effective.right_us));
  text("#manual-throttle-slew", fmt(state.control.limits.manual_slew_per_s, 2, "/s"));
  text("#manual-throttle-output", fmt(num(effective.throttle) ?? manualOutput.throttle, 2));
  if (state.control.mode !== "manual") {
    text("#manual-throttle-input", fmt(manualInput.throttle ?? 0, 2));
  }
  if (!state.control.stopRetry) {
    setReason(manual.reason || "drive off", null);
  }
  updateStaleness(manual);
  updateActuatorBanner(manual);
}

function updateStaleness(manual) {
  const fillEl = $("#staleness-fill");
  const labelEl = $("#staleness-label");
  if (!fillEl || !labelEl) return;
  const stale = manual?.stale === true;
  const ageS = num(manual?.age_s);
  const limit = state.control.limits.stale_after_s || 0.45;
  if (state.control.mode !== "manual" || ageS === null) {
    fillEl.style.setProperty("--fill", "0%");
    labelEl.textContent = state.control.mode === "manual" ? "armed" : "--";
    return;
  }
  if (stale) {
    fillEl.style.setProperty("--fill", "100%");
    labelEl.textContent = "stale";
    return;
  }
  const remaining = Math.max(0, limit - ageS);
  const pct = Math.max(0, Math.min(1, remaining / limit));
  fillEl.style.setProperty("--fill", `${(pct * 100).toFixed(0)}%`);
  labelEl.textContent = `${remaining.toFixed(2)}s`;
}

function updateActuatorBanner(manual) {
  const banner = $("#manual-banner");
  if (!banner) return;
  const actuator = typeof manual?.actuator === "string" ? manual.actuator : null;
  if (state.control.mode === "auto") {
    return;
  }
  if (actuator === "error") {
    banner.hidden = false;
    banner.dataset.tone = "error";
    banner.textContent = `Actuator error: ${manual?.effective?.error || "unknown"}`;
  } else {
    banner.hidden = true;
    banner.removeAttribute("data-tone");
    banner.textContent = "";
  }
}

function setZone(selector, value) {
  const node = $(selector);
  if (!node) return;
  node.style.width = `${clamp(num(value) ?? 0) * 100}%`;
}

function setBar(selector, value, min, max, color) {
  const node = $(selector);
  if (!node) return;
  const n = num(value);
  const pct = n === null ? 0 : (n - min) / (max - min);
  node.style.setProperty("--value", `${clamp(pct) * 100}%`);
  node.style.setProperty("--bar-color", color);
}

function canvasContext(id) {
  const canvas = document.getElementById(id);
  if (!canvas) return null;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return null;
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = Math.max(1, Math.round(rect.width));
  const cssHeight = Math.max(1, Math.round(rect.height));
  const targetWidth = Math.round(cssWidth * dpr);
  const targetHeight = Math.round(cssHeight * dpr);
  if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
    canvas.width = targetWidth;
    canvas.height = targetHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { canvas, ctx, w: cssWidth, h: cssHeight };
}

window.addEventListener("resize", () => requestAnimationFrame(() => drawAll(state.last)));

function clearCanvas(ctx, w, h, color = colors.bg) {
  ctx.fillStyle = color;
  ctx.fillRect(0, 0, w, h);
}

function drawGrid(ctx, w, h, step, color = colors.grid) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  for (let x = step; x < w; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = step; y < h; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
}

function drawLabel(ctx, textValue, x, y, color = colors.ink, size = 13) {
  ctx.fillStyle = color;
  ctx.font = `900 ${size}px Bahnschrift, Segoe UI, sans-serif`;
  ctx.fillText(textValue, x, y);
}

function drawRadar(id, data, detail = false) {
  const surface = canvasContext(id);
  if (!surface) return;
  const { ctx, w, h } = surface;
  const mmwave = data?.mmwave || {};
  const raw = mmwave.raw_points || [];
  const filtered = getPoints(mmwave);
  const clusters = mmwave.clusters || [];
  const scores = mmwave.scores || mmwave.zones || {};
  const health = healthOf(data, "mmwave");
  const rangeY = detail ? 3.4 : 3.0;
  const rangeX = detail ? 1.8 : 1.55;
  const pad = detail ? 34 : 26;

  clearCanvas(ctx, w, h);
  drawGrid(ctx, w, h, detail ? 44 : 38);

  ctx.strokeStyle = colors.line;
  ctx.lineWidth = 2;
  ctx.strokeRect(1, 1, w - 2, h - 2);

  const toScreen = (point) => ({
    x: w / 2 + (point.x / rangeX) * (w / 2 - pad),
    y: h - pad - (point.y / rangeY) * (h - pad * 2),
  });

  ctx.fillStyle = "rgba(217,79,92,0.12)";
  ctx.fillRect(w * 0.39, 0, w * 0.22, h);
  ctx.strokeStyle = colors.red;
  ctx.lineWidth = 2;
  ctx.strokeRect(w * 0.39, 0, w * 0.22, h);

  drawRangeRings(ctx, w, h, pad, rangeY);

  raw.forEach((point) => {
    const p = toScreen(point);
    if (p.x < 0 || p.x > w || p.y < 0 || p.y > h) return;
    ctx.fillStyle = colors.dead;
    ctx.fillRect(p.x - 2, p.y - 2, 4, 4);
  });

  filtered.forEach((point) => {
    const p = toScreen(point);
    if (p.x < 0 || p.x > w || p.y < 0 || p.y > h) return;
    const snr = pointSnr(point) ?? 80;
    const radius = detail ? clamp(snr / 80, 3, 7) : clamp(snr / 90, 3, 6);
    ctx.fillStyle = point.y < 0.7 ? colors.red : snr > 220 ? colors.green : colors.teal;
    ctx.beginPath();
    ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
    ctx.fill();
  });

  clusters.forEach((cluster) => {
    const p = toScreen({ x: cluster.cx || 0, y: cluster.cy || 0 });
    if (p.x < -30 || p.x > w + 30 || p.y < -30 || p.y > h + 30) return;
    const color = cluster.zone === "front" ? colors.red : cluster.zone === "left" ? colors.yellow : cluster.zone === "right" ? colors.orange : colors.ink;
    ctx.strokeStyle = color;
    ctx.lineWidth = cluster.is_singleton ? 2 : 4;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9 + clamp(cluster.confidence || 0, 0, 1) * 18, 0, Math.PI * 2);
    ctx.stroke();
    if (detail) drawLabel(ctx, `${cluster.count || 1}`, p.x + 10, p.y - 10, color, 12);
  });

  drawBoat(ctx, w / 2, h - pad + 8, detail ? 24 : 19, mmwave.control?.steering);
  drawLabel(ctx, "LEFT", 14, 22, colors.yellow, 12);
  drawLabel(ctx, "FRONT", w / 2 - 25, 22, colors.red, 12);
  drawLabel(ctx, "RIGHT", w - 58, 22, colors.orange, 12);

  drawScorePanel(ctx, w, h, scores, detail);

  if (health !== "live") {
    ctx.fillStyle = "rgba(255,253,244,0.45)";
    ctx.fillRect(0, 0, w, h);
  }
}

function drawRangeRings(ctx, w, h, pad, rangeY) {
  ctx.strokeStyle = colors.line;
  ctx.lineWidth = 1;
  for (let meter = 0.5; meter <= rangeY; meter += 0.5) {
    const y = h - pad - (meter / rangeY) * (h - pad * 2);
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
    drawLabel(ctx, `${meter.toFixed(1)}m`, 8, y - 4, colors.muted, 10);
  }
}

function drawBoat(ctx, x, y, size, steering = 0) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate((num(steering) ?? 0) * 0.35);
  ctx.fillStyle = colors.ink;
  ctx.beginPath();
  ctx.moveTo(0, -size);
  ctx.lineTo(-size * 0.58, size * 0.72);
  ctx.lineTo(0, size * 0.42);
  ctx.lineTo(size * 0.58, size * 0.72);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = colors.teal;
  ctx.fillRect(-size * 0.18, -size * 0.08, size * 0.36, size * 0.34);
  ctx.restore();
}

function drawScorePanel(ctx, w, h, scores, detail) {
  const width = detail ? 150 : 116;
  const x = w - width - 14;
  const y = h - (detail ? 112 : 92);
  const rows = [
    ["L", scores.left, colors.yellow],
    ["F", scores.front, colors.red],
    ["R", scores.right, colors.orange],
  ];
  ctx.fillStyle = colors.panel;
  ctx.fillRect(x, y, width, detail ? 94 : 74);
  ctx.strokeStyle = colors.line;
  ctx.strokeRect(x, y, width, detail ? 94 : 74);
  rows.forEach(([label, value, color], index) => {
    const rowY = y + 18 + index * (detail ? 24 : 18);
    drawLabel(ctx, label, x + 8, rowY + 4, color, 11);
    ctx.strokeStyle = colors.dead;
    ctx.strokeRect(x + 28, rowY - 7, width - 42, 10);
    ctx.fillStyle = color;
    ctx.fillRect(x + 28, rowY - 7, clamp(num(value) ?? 0) * (width - 42), 10);
  });
}

function drawPosition(id, data, compact = false) {
  const surface = canvasContext(id);
  if (!surface) return;
  const { ctx, w, h } = surface;
  const gnss = data?.gnss || {};
  const health = healthOf(data, "gnss");
  clearCanvas(ctx, w, h);
  drawGrid(ctx, w, h, compact ? 24 : 40, colors.grid);

  ctx.strokeStyle = colors.teal;
  ctx.strokeRect(1, 1, w - 2, h - 2);

  const points = scaleTrack(state.gnss, w, h, compact ? 18 : 34);
  if (points.length > 1) {
    ctx.strokeStyle = colors.green;
    ctx.lineWidth = compact ? 2 : 4;
    ctx.beginPath();
    points.forEach((point, index) => {
      if (index === 0) ctx.moveTo(point.x, point.y);
      else ctx.lineTo(point.x, point.y);
    });
    ctx.stroke();
  }

  const last = points[points.length - 1] || { x: w / 2, y: h / 2 };
  drawHeadingArrow(ctx, last.x, last.y, compact ? 14 : 22, num(gnss.heading_deg));
  drawLabel(ctx, `${fmt(gnss.speed_mps, 2)} m/s`, 12, h - 14, colors.ink, compact ? 12 : 16);

  if (health !== "live") drawEmptyHint(ctx, w, h, "No GNSS fix");
}

function scaleTrack(track, w, h, pad) {
  if (!track.length) return [];
  const lats = track.map((point) => point.lat);
  const lons = track.map((point) => point.lon);
  const minLat = Math.min(...lats);
  const maxLat = Math.max(...lats);
  const minLon = Math.min(...lons);
  const maxLon = Math.max(...lons);
  const latSpan = Math.max(maxLat - minLat, 0.00002);
  const lonSpan = Math.max(maxLon - minLon, 0.00002);
  return track.map((point) => ({
    x: pad + ((point.lon - minLon) / lonSpan) * (w - pad * 2),
    y: h - pad - ((point.lat - minLat) / latSpan) * (h - pad * 2),
  }));
}

function drawHeadingArrow(ctx, x, y, size, heading) {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(((heading ?? 0) * Math.PI) / 180);
  ctx.fillStyle = colors.yellow;
  ctx.beginPath();
  ctx.moveTo(0, -size);
  ctx.lineTo(size * 0.62, size);
  ctx.lineTo(0, size * 0.54);
  ctx.lineTo(-size * 0.62, size);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawCompass(id, data) {
  const surface = canvasContext(id);
  if (!surface) return;
  const { ctx, w, h } = surface;
  const heading = num(data?.gnss?.heading_deg);
  const health = healthOf(data, "gnss");
  const r = Math.min(w, h) / 2 - 18;
  clearCanvas(ctx, w, h);
  ctx.strokeStyle = colors.line;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(w / 2, h / 2, r, 0, Math.PI * 2);
  ctx.stroke();
  ["N", "E", "S", "W"].forEach((label, index) => {
    const angle = (index * Math.PI) / 2 - Math.PI / 2;
    drawLabel(ctx, label, w / 2 + Math.cos(angle) * (r - 18) - 6, h / 2 + Math.sin(angle) * (r - 18) + 5, colors.ink, 15);
  });
  if (health === "live" && heading !== null) {
    drawHeadingArrow(ctx, w / 2, h / 2, r - 34, heading);
    drawLabel(ctx, fmt(heading, 0, " deg"), w / 2 - 34, h / 2 + r + 4, colors.yellow, 17);
  } else {
    drawEmptyHint(ctx, w, h, "Heading unavailable");
  }
}

function drawEmptyHint(ctx, w, h, label) {
  ctx.fillStyle = "rgba(255,253,244,0.58)";
  ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = colors.muted;
  ctx.font = "900 14px Bahnschrift, Segoe UI, sans-serif";
  ctx.fillText(label, 18, 30);
}

function drawAttitude(id, data, large = false) {
  const surface = canvasContext(id);
  if (!surface) return;
  const { ctx, w, h } = surface;
  const imu = data?.imu || {};
  const ax = num(imu.accel_x_g) ?? 0;
  const ay = num(imu.accel_y_g) ?? 0;
  const az = num(imu.accel_z_g) ?? 1;
  const yaw = num(imu.yaw_relative_deg) ?? 0;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - (large ? 28 : 14);
  const pitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az));
  const roll = Math.atan2(ay, az);
  const dotX = cx + clamp(roll / 0.75, -1, 1) * r * 0.72;
  const dotY = cy + clamp(pitch / 0.75, -1, 1) * r * 0.72;

  clearCanvas(ctx, w, h);
  ctx.strokeStyle = colors.line;
  ctx.lineWidth = large ? 3 : 2;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();

  ctx.strokeStyle = colors.dead;
  ctx.beginPath();
  ctx.moveTo(cx - r, cy);
  ctx.lineTo(cx + r, cy);
  ctx.moveTo(cx, cy - r);
  ctx.lineTo(cx, cy + r);
  ctx.stroke();

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate((yaw * Math.PI) / 180);
  ctx.strokeStyle = colors.teal;
  ctx.lineWidth = large ? 5 : 3;
  ctx.beginPath();
  ctx.moveTo(0, -r * 0.82);
  ctx.lineTo(0, r * 0.82);
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = colors.red;
  ctx.beginPath();
  ctx.arc(dotX, dotY, large ? 14 : 8, 0, Math.PI * 2);
  ctx.fill();
  drawLabel(ctx, `P ${((pitch * 180) / Math.PI).toFixed(1)}`, large ? 24 : 10, h - (large ? 46 : 22), colors.yellow, large ? 18 : 11);
  drawLabel(ctx, `R ${((roll * 180) / Math.PI).toFixed(1)}`, large ? 24 : 10, h - (large ? 22 : 8), colors.teal, large ? 18 : 11);
  drawLabel(ctx, `Y ${yaw.toFixed(1)}`, cx - (large ? 38 : 26), cy + 5, colors.ink, large ? 20 : 12);
}

function drawTrace(id, values, options = {}) {
  const surface = canvasContext(id);
  if (!surface) return;
  const { ctx, w, h } = surface;
  const color = options.color || colors.green;
  const min = options.min ?? 0;
  const max = options.max ?? 1;
  clearCanvas(ctx, w, h);
  drawGrid(ctx, w, h, options.step || 24, colors.grid);
  ctx.strokeStyle = colors.line;
  ctx.strokeRect(1, 1, w - 2, h - 2);

  if (values.length < 2) {
    drawLabel(ctx, "NO TRACE", 10, 22, colors.dead, 12);
    return;
  }

  ctx.strokeStyle = color;
  ctx.lineWidth = options.width || 3;
  ctx.beginPath();
  values.forEach((value, index) => {
    const x = 8 + (index / Math.max(values.length - 1, 1)) * (w - 16);
    const y = h - 8 - clamp((value - min) / (max - min)) * (h - 16);
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function drawWheel() {
  const surface = canvasContext("manual-wheel");
  if (!surface) return;
  const { ctx, w, h } = surface;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) / 2 - 6;
  const angleRad = (state.control.wheelAngleDeg * Math.PI) / 180;
  const enabled = isControlEnabled();

  ctx.fillStyle = colors.bg;
  ctx.fillRect(0, 0, w, h);

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(angleRad);

  ctx.lineWidth = Math.max(10, r * 0.13);
  ctx.strokeStyle = enabled ? colors.ink : colors.dead;
  ctx.beginPath();
  ctx.arc(0, 0, r - ctx.lineWidth / 2, 0, Math.PI * 2);
  ctx.stroke();

  ctx.fillStyle = enabled ? colors.yellow : "#e5d9b5";
  ctx.beginPath();
  ctx.arc(0, 0, r * 0.32, 0, Math.PI * 2);
  ctx.fill();

  ctx.lineWidth = Math.max(8, r * 0.085);
  ctx.lineCap = "round";
  ctx.strokeStyle = enabled ? colors.ink : colors.dead;
  for (let i = 0; i < 3; i += 1) {
    const a = (i * Math.PI * 2) / 3 - Math.PI / 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(a) * (r - 14), Math.sin(a) * (r - 14));
    ctx.stroke();
  }

  ctx.fillStyle = enabled ? colors.red : "#c89aa1";
  const indW = r * 0.16;
  const indH = r * 0.34;
  roundedRect(ctx, -indW / 2, -r + 4, indW, indH, indW / 3);
  ctx.fill();

  ctx.fillStyle = enabled ? colors.ink : colors.dead;
  ctx.beginPath();
  ctx.arc(0, 0, r * 0.12, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();

  ctx.strokeStyle = colors.dead;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx - 6, cy + r + 4);
  ctx.lineTo(cx + 6, cy + r + 4);
  ctx.stroke();

  drawLabel(ctx, `${state.control.wheelAngleDeg >= 0 ? "+" : ""}${state.control.wheelAngleDeg.toFixed(0)}°`, cx - 22, cy + r + 22, colors.muted, 12);
}

function roundedRect(ctx, x, y, w, h, r) {
  const radius = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.lineTo(x + w - radius, y);
  ctx.quadraticCurveTo(x + w, y, x + w, y + radius);
  ctx.lineTo(x + w, y + h - radius);
  ctx.quadraticCurveTo(x + w, y + h, x + w - radius, y + h);
  ctx.lineTo(x + radius, y + h);
  ctx.quadraticCurveTo(x, y + h, x, y + h - radius);
  ctx.lineTo(x, y + radius);
  ctx.quadraticCurveTo(x, y, x + radius, y);
  ctx.closePath();
}

function drawThrottle() {
  const surface = canvasContext("manual-throttle");
  if (!surface) return;
  const { ctx, w, h } = surface;
  const padding = 14;
  const trackTop = padding;
  const trackBottom = h - padding;
  const trackHeight = trackBottom - trackTop;
  const trackLeft = padding;
  const trackRight = w - padding;
  const trackWidth = trackRight - trackLeft;
  const intent = state.control.intent.throttle;
  const enabled = isControlEnabled();

  ctx.fillStyle = "#f7efd9";
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = colors.dead;
  ctx.lineWidth = 1;
  for (const tick of [0.25, 0.5, 0.75]) {
    const y = trackBottom - tick * trackHeight;
    ctx.beginPath();
    ctx.moveTo(trackLeft, y);
    ctx.lineTo(trackRight, y);
    ctx.stroke();
    drawLabel(ctx, `${(tick * 100).toFixed(0)}`, trackLeft + 4, y - 4, colors.muted, 10);
  }

  const fillH = clamp(intent, 0, 1) * trackHeight;
  const fillY = trackBottom - fillH;
  const grad = ctx.createLinearGradient(0, trackBottom, 0, trackTop);
  grad.addColorStop(0, enabled ? colors.green : "#bcd1ad");
  grad.addColorStop(1, enabled ? colors.yellow : "#e6d5a8");
  ctx.fillStyle = grad;
  ctx.fillRect(trackLeft, fillY, trackWidth, fillH);

  const thumbY = clamp(fillY, trackTop - 1, trackBottom + 1);
  ctx.fillStyle = enabled ? colors.ink : colors.dead;
  roundedRect(ctx, trackLeft - 2, thumbY - 9, trackWidth + 4, 14, 4);
  ctx.fill();

  ctx.fillStyle = "#fffdf4";
  ctx.fillRect(trackLeft + 6, thumbY - 2, trackWidth - 12, 2);

  drawLabel(ctx, fmt(intent, 2), trackLeft + 6, trackTop + 16, colors.muted, 11);
}

function drawAll(data) {
  drawWheel();
  drawThrottle();
  if (!data) return;
  drawRadar("radar-overview", data, false);
  drawRadar("radar-detail", data, true);
  drawPosition("position-overview", data, true);
  drawPosition("gnss-detail", data, false);
  drawCompass("gnss-compass", data);
  drawAttitude("attitude-overview", data, false);
  drawAttitude("imu-attitude", data, true);
  drawTrace("throttle-overview", state.throttle, { color: colors.green, min: 0, max: 1, step: 22, width: 2 });
  drawTrace("accel-overview", state.accel, { color: colors.yellow, min: 0.85, max: 1.2, step: 22, width: 2 });
  drawTrace("imu-accel-trace", state.accel, { color: colors.yellow, min: 0.75, max: 1.35, step: 42, width: 4 });
}

function applySnapshot(data) {
  state.last = data;
  updateHistory(data);
  updateDom(data);
  drawAll(data);
}

function connectEvents() {
  const events = new EventSource("/events");
  events.onmessage = (event) => {
    try {
      applySnapshot(JSON.parse(event.data));
      state.control.sse.everConnected = true;
      state.control.sse.lastEventAt = performance.now();
      state.control.serverReachable = true;
      updateDegradedState();
    } catch (error) {
      console.error(error);
    }
  };
  events.onerror = () => {
    text("#stream-label", "stream offline");
    text("#stream-meta", "retrying");
    $(".stream-card")?.setAttribute("data-health", "error");
    state.control.serverReachable = false;
    updateDegradedState();
  };
}

function updateDegradedState() {
  if (!controlSurface) return;
  const reachable = state.control.serverReachable;
  controlSurface.setAttribute("data-degraded", reachable ? "false" : "true");
  const overlay = controlSurface.querySelector(".comms-overlay");
  if (overlay) overlay.hidden = reachable;
  if (!reachable && state.control.mode === "manual") {
    setReason("comms lost - controls locked", "error");
  }
}

function watchdogTick() {
  if (state.control.sse.everConnected && performance.now() - state.control.sse.lastEventAt > 1500) {
    state.control.serverReachable = false;
    updateDegradedState();
  }
}

fetch("/api/snapshot")
  .then((response) => response.json())
  .then(applySnapshot)
  .catch(() => {
    text("#stream-label", "snapshot failed");
    $(".stream-card")?.setAttribute("data-health", "error");
  })
  .finally(connectEvents);

bindManualControls();
syncModePills();
setInterval(() => sendManualCommand(false), 50);
setInterval(watchdogTick, 500);
window.addEventListener("resize", () => {
  drawWheel();
  drawThrottle();
});
