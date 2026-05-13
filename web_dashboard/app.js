const tabs = document.querySelectorAll(".tab");
const views = document.querySelectorAll(".view");
const connectionDot = document.querySelector("#connection-dot");
const connectionLabel = document.querySelector("#connection-label");
const track = [];

tabs.forEach((tab) => {
  tab.addEventListener("click", () => {
    tabs.forEach((item) => item.classList.remove("active"));
    views.forEach((item) => item.classList.remove("active"));
    tab.classList.add("active");
    document.querySelector(`#${tab.dataset.tab}`).classList.add("active");
  });
});

function setText(id, value) {
  const node = document.querySelector(id);
  if (node) node.textContent = value;
}

function fmt(value, suffix = "") {
  if (value === null || value === undefined || Number.isNaN(value)) return "--";
  return `${value}${suffix}`;
}

function pointSnr(point) {
  return point.snr ?? point.snr_raw ?? 0;
}

function drawRadar(canvasId, data) {
  const canvas = document.querySelector(canvasId);
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#102b31";
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "rgba(198, 231, 221, 0.16)";
  ctx.lineWidth = 1;
  for (let i = 1; i < 5; i += 1) {
    const y = h - (i / 5) * h;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  for (let i = -2; i <= 2; i += 1) {
    const x = w / 2 + (i / 3) * (w / 2);
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }

  ctx.fillStyle = "#d38b24";
  ctx.beginPath();
  ctx.moveTo(w / 2, h - 20);
  ctx.lineTo(w / 2 - 18, h - 2);
  ctx.lineTo(w / 2 + 18, h - 2);
  ctx.closePath();
  ctx.fill();

  const rawPoints = data.mmwave.raw_points || [];
  const points = data.mmwave.points || data.mmwave.filtered_points || [];
  rawPoints.forEach((point) => {
    const x = w / 2 + (point.x / 1.5) * (w / 2 - 34);
    const y = h - 28 - (point.y / 3.0) * (h - 58);
    ctx.fillStyle = "rgba(255, 250, 240, 0.22)";
    ctx.beginPath();
    ctx.arc(x, y, 3, 0, Math.PI * 2);
    ctx.fill();
  });
  points.forEach((point) => {
    const x = w / 2 + (point.x / 1.5) * (w / 2 - 34);
    const y = h - 28 - (point.y / 3.0) * (h - 58);
    const snr = pointSnr(point);
    const radius = 3 + Math.min(snr / 110, 4);
    ctx.fillStyle = point.y < 0.75 ? "#bc3d35" : snr > 220 ? "#9fe3bf" : "#5db4c5";
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.fill();
  });
  (data.mmwave.clusters || []).forEach((cluster) => {
    const x = w / 2 + (cluster.cx / 1.5) * (w / 2 - 34);
    const y = h - 28 - (cluster.cy / 3.0) * (h - 58);
    const radius = 8 + Math.min((cluster.confidence || 0) * 18, 18);
    ctx.strokeStyle = cluster.zone === "front" ? "#bc3d35" : "#d38b24";
    ctx.lineWidth = cluster.is_singleton ? 2 : 4;
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, Math.PI * 2);
    ctx.stroke();
  });

  ctx.fillStyle = "rgba(255, 250, 240, 0.82)";
  ctx.font = "16px Aptos, Segoe UI, sans-serif";
  ctx.fillText("left", 18, 28);
  ctx.fillText("front", w / 2 - 20, 28);
  ctx.fillText("right", w - 58, 28);
}

function drawTrack(data) {
  const canvas = document.querySelector("#track-canvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#e8f1ed";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "rgba(18, 51, 58, 0.16)";
  for (let x = 0; x < w; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  if (typeof data.gnss.lat === "number" && typeof data.gnss.lon === "number") {
    track.push([data.gnss.lat, data.gnss.lon]);
  }
  if (track.length > 160) track.shift();
  const origin = track[0] || [data.gnss.lat, data.gnss.lon];
  const scale = 900000;
  const points = track.map(([lat, lon]) => [w / 2 + (lon - origin[1]) * scale, h / 2 - (lat - origin[0]) * scale]);

  ctx.strokeStyle = "#1f8a5b";
  ctx.lineWidth = 3;
  ctx.beginPath();
  points.forEach(([x, y], index) => {
    if (index === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  if (points.length > 0) {
    const last = points[points.length - 1];
    ctx.fillStyle = "#d38b24";
    ctx.beginPath();
    ctx.arc(last[0], last[1], 8, 0, Math.PI * 2);
    ctx.fill();
  } else {
    ctx.fillStyle = "#627069";
    ctx.font = "22px Aptos, Segoe UI, sans-serif";
    ctx.fillText("GNSS unavailable", 28, 44);
  }
}

function renderRanges(containerId, readings, maxValue, unit) {
  const container = document.querySelector(containerId);
  container.innerHTML = "";
  if (!readings || Object.keys(readings).length === 0) {
    const item = document.createElement("article");
    item.className = "range-item unavailable";
    item.innerHTML = "<strong>--</strong><span>feed unavailable</span>";
    container.append(item);
    return;
  }
  Object.entries(readings).forEach(([name, value]) => {
    const item = document.createElement("article");
    item.className = "range-item";
    const shell = document.createElement("div");
    shell.className = "bar-shell";
    const fill = document.createElement("div");
    fill.className = "bar-fill";
    fill.style.height = `${Math.max(4, Math.min(100, (value / maxValue) * 100))}%`;
    shell.append(fill);
    item.append(shell);
    item.insertAdjacentHTML("beforeend", `<strong>${value} ${unit}</strong><span>${name}</span>`);
    container.append(item);
  });
}

function update(data) {
  connectionDot.classList.add("live");
  connectionLabel.textContent = `${data.mode} stream`;
  setText("#health-mmwave", data.health.mmwave);
  setText("#health-gnss", data.health.gnss);
  setText("#health-sonar", data.health.sonar);
  setText("#health-ultrasonic", data.health.ultrasonic);
  setText("#command-pill", data.mmwave.command.replace("_", " "));
  setText("#fix-pill", data.gnss.fix);
  setText("#lat", fmt(data.gnss.lat));
  setText("#lon", fmt(data.gnss.lon));
  setText("#speed", fmt(data.gnss.speed_mps, " m/s"));
  setText("#heading", fmt(data.gnss.heading_deg, " deg"));
  setText("#gnss-fix", data.gnss.fix);
  setText("#hdop", fmt(data.gnss.hdop));
  setText("#gnss-speed", fmt(data.gnss.speed_mps, " m/s"));
  setText("#gnss-heading", fmt(data.gnss.heading_deg, " deg"));
  setText("#sat-pill", `${data.gnss.satellites} sats`);
  const radarPoints = data.mmwave.points || data.mmwave.filtered_points || [];
  setText("#point-count", `${radarPoints.length} points`);
  const scores = data.mmwave.scores || data.mmwave.zones || {};
  document.querySelector("#zone-left").value = scores.left || 0;
  document.querySelector("#zone-front").value = scores.front || 0;
  document.querySelector("#zone-right").value = scores.right || 0;
  const counts = data.mmwave.point_count || {};
  setText("#mmwave-frame", fmt(data.mmwave.frame_number));
  setText("#mmwave-raw", fmt(counts.raw ?? (data.mmwave.raw_points || []).length));
  setText("#mmwave-filtered", fmt(counts.filtered ?? radarPoints.length));
  setText("#mmwave-clusters", fmt(counts.clusters ?? (data.mmwave.clusters || []).length));
  const control = data.mmwave.control || {};
  setText("#mmwave-throttle", fmt(control.throttle));
  setText("#mmwave-target-throttle", fmt(control.target_throttle));
  setText("#mmwave-steering", fmt(control.steering));
  setText("#mmwave-target-steering", fmt(control.target_steering));
  setText("#mmwave-reason", data.mmwave.reason || "--");
  document.querySelector("#boat-marker").style.transform = `translate(-50%, -50%) rotate(${data.gnss.heading_deg || 0}deg)`;

  drawRadar("#radar-overview", data);
  drawRadar("#radar-detail", data);
  drawTrack(data);
  renderRanges("#sonar-bars", data.sonar, 2.5, "m");
  renderRanges("#ultrasonic-bars", data.ultrasonic, 110, "cm");
}

const events = new EventSource("/events");
events.onmessage = (event) => update(JSON.parse(event.data));
events.onerror = () => {
  connectionDot.classList.remove("live");
  connectionLabel.textContent = "stream offline";
};
