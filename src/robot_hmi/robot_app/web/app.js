// robot_control_suite teleop console front end. No framework, no build step. All text from the server is
// inserted with textContent (log lines and error messages may contain anything).
"use strict";

const ROLE_TITLES = { head: "头部", left_wrist: "左腕", right_wrist: "右腕" };
let lastState = null;

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") node.className = v;
    else if (k.startsWith("on")) node.addEventListener(k.slice(2), v);
    else node.setAttribute(k, v);
  }
  for (const c of children) node.append(c instanceof Node ? c : document.createTextNode(String(c ?? "")));
  return node;
}

function toast(message, isError = false) {
  const t = document.getElementById("toast");
  t.textContent = message;
  t.className = "toast" + (isError ? " error" : "");
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => { t.className = "toast hidden"; }, 5000);
}

async function api(path, { method = "GET", body } = {}) {
  const res = await fetch(path, {
    method,
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  let data = null;
  try { data = await res.json(); } catch (_) { /* empty body */ }
  if (!res.ok) {
    const detail = (data && data.detail) || {};
    const err = new Error(detail.message || `${res.status} ${res.statusText}`);
    err.status = res.status;
    err.detail = detail;
    throw err;
  }
  return data;
}

function confirmDialog(text) {
  return new Promise((resolve) => {
    const dialog = document.getElementById("confirm");
    document.getElementById("confirm-text").textContent = text;
    dialog.addEventListener("close", () => resolve(dialog.returnValue === "ok"), { once: true });
    dialog.showModal();
  });
}

// Stop / restart: the server answers 409 need_confirm when the controllers are not in HOLD.
async function guarded(path, label) {
  try {
    await api(path, { method: "POST", body: { confirm: false } });
    toast(`${label}：完成`);
  } catch (err) {
    if (err.status === 409 && err.detail.need_confirm) {
      if (await confirmDialog(err.detail.message)) {
        try {
          await api(path, { method: "POST", body: { confirm: true } });
          toast(`${label}：已切到 HOLD 并完成`);
        } catch (err2) { toast(`${label}失败：${err2.message}`, true); }
      }
    } else {
      toast(`${label}失败：${err.message}`, true);
    }
  }
  refresh();
}

async function action(path, label) {
  try {
    await api(path, { method: "POST" });
    toast(`${label}：完成`);
  } catch (err) { toast(`${label}失败：${err.message}`, true); }
  refresh();
}

async function exportDiag(button) {
  button.disabled = true;
  try {
    const { name } = await api("/api/diag", { method: "POST" });
    toast("诊断包已生成，正在下载");
    window.location.href = `/api/diag/${encodeURIComponent(name)}`;
  } catch (err) { toast(`导出诊断包失败：${err.message}`, true); }
  button.disabled = false;
}
document.getElementById("diag").addEventListener("click", (event) => exportDiag(event.currentTarget));

function fmtUptime(s) {
  if (s == null) return "-";
  s = Math.floor(s);
  const pad = (n) => String(n).padStart(2, "0");
  return `${pad(Math.floor(s / 3600))}:${pad(Math.floor((s % 3600) / 60))}:${pad(s % 60)}`;
}

function fmtRates(rates) {
  return Object.entries(rates || {})
    .map(([topic, hz]) => `${topic.split("/").pop()} ${hz == null ? "-" : hz + " Hz"}`).join("  ");
}

function renderUnits(state) {
  const body = document.getElementById("units");
  body.replaceChildren(...state.units.map((u) => el("tr", { class: u.running ? "running" : "stopped" },
    el("td", {}, el("strong", {}, u.title), el("div", { class: "muted" }, u.name)),
    el("td", {}, el("span", { class: "dot" }), u.running ? "运行" : "停止",
      u.pid ? el("div", { class: "muted" }, `pid ${u.pid}`) : ""),
    el("td", {}, fmtUptime(u.uptime_s)),
    el("td", { class: "mono" }, fmtRates(u.rates)),
    el("td", { class: "error-cell", title: u.last_error || "" }, u.last_error || ""),
    el("td", { class: "actions" },
      el("button", { class: "primary", onclick: () => action(`/api/units/${u.name}/start`, `启动${u.title}`) }, "启动"),
      el("button", { class: "danger", onclick: () => guarded(`/api/units/${u.name}/stop`, `停止${u.title}`) }, "停止"),
      el("button", { onclick: () => guarded(`/api/units/${u.name}/restart`, `重启${u.title}`) }, "重启"),
      el("button", { onclick: () => { document.getElementById("log-unit").value = u.name; loadLog(); } }, "日志")),
  )));

  const select = document.getElementById("log-unit");
  const current = select.value;
  const names = state.units.map((u) => u.name);
  if (select.options.length !== names.length || names.some((n, i) => select.options[i].value !== n)) {
    select.replaceChildren(...state.units.map((u) => el("option", { value: u.name }, `${u.title}（${u.name}）`)));
    if (names.includes(current)) select.value = current;
  }
}

// Addresses for other computers (e.g. the 4090); only shown when the console was started with --host.
function renderAccess(state) {
  const box = document.getElementById("access");
  const urls = state.access_urls || [];
  box.className = urls.length ? "access" : "access hidden";
  if (!urls.length) return;
  box.replaceChildren("其他电脑访问：", ...urls.flatMap((u) => [el("a", { href: u }, u), "　"]));
}

// XR is started together with the controllers but only the headset can connect it: tell the operator what to check.
const XR_WAIT_S = 10;
function xrHint(state) {
  const xr = (state.units || []).find((u) => u.name === "xr");
  if (!xr || !xr.running || state.xr_connected === true || (xr.uptime_s || 0) < XR_WAIT_S) return "";
  const pc = (state.units || []).find((u) => u.name === "pc_service");
  if (pc && !pc.running) return "XR 已启动但 PC Service 没有运行：请在模块里启动 pc_service，或查看它的日志";
  return `XR 服务已启动，正在等头显连接（${XR_WAIT_S} 秒内没有收到）：请在头显的 XRoboToolkit 里手动连接这台电脑并开启 Send；仍不行请查看 xr 日志`;
}

// The live status of the parts that matter: controller, FSM, PC Service, XR, cameras (the server judges them).
const LEVEL_CLASS = { ok: " ok", warn: " warn", bad: " bad", off: " off" };
function renderSummary(state) {
  document.getElementById("summary").replaceChildren(...(state.summary || []).map((part) =>
    el("span", { class: "pill" + (LEVEL_CLASS[part.level] || ""), title: part.detail || "" },
      `${part.label}：${part.text}`, part.unit ? unitButton(part) : "")));
  // A start / stop / restart takes seconds: show which step runs and keep the buttons from being pressed twice.
  const activity = state.activity;
  document.getElementById("activity").textContent = activity
    ? `正在${activity.action}：${activity.step}（${activity.index}/${activity.total}）` : "";
  for (const id of ["inference-toggle", "all-toggle"]) document.getElementById(id).disabled = !!activity;
  for (const button of document.querySelectorAll(".chip-btn")) button.disabled = !!activity;
}

// One module on its own: start it when it is stopped, restart it otherwise (the controller asks again outside HOLD).
function unitButton(part) {
  const stopped = part.level === "off";
  return el("button", {
    class: "chip-btn", title: stopped ? `启动${part.label}` : `重启${part.label}`,
    onclick: async () => {
      if (stopped) { action(`/api/units/${part.unit}/start`, `启动${part.label}`); return; }
      const extra = part.unit === "control" ? "\n运控会先抱闸再重新上电，RViz、手柄遥操会一起重启。请确认手臂周围无人。" : "";
      if (await confirmDialog(`重启${part.label}？${extra}`)) guarded(`/api/units/${part.unit}/restart`, `重启${part.label}`);
    },
  }, stopped ? "▶" : "↻");
}

// The XR section: the headset state (the key table is static).
function renderXr(state) {
  const xr = (state.summary || []).find((part) => part.key === "xr");
  const pill = document.getElementById("xr-state");
  pill.textContent = xr ? xr.text : "-";
  pill.className = "pill" + (xr ? (LEVEL_CLASS[xr.level] || "") : "");
  pill.title = xr && xr.detail ? xr.detail : "";
}

function renderStatus(state) {
  renderSummary(state);
  renderXr(state);
  const banner = document.getElementById("banner");
  const messages = [state.config_error && `配置：${state.config_error}`, state.probe_error, xrHint(state)].filter(Boolean);
  banner.textContent = messages.join("　|　");
  banner.className = "banner" + (messages.length ? "" : " hidden");
}

// Previews: one <img> per running camera; the MJPEG stream ends after 5 s without frames and is retried.
function renderPreviews(state) {
  const box = document.getElementById("previews");
  const cams = state.units.filter((u) => u.is_camera);
  box.classList.toggle("off", !previewOn());
  for (const cam of cams) {
    let card = document.getElementById(`preview-${cam.name}`);
    if (!card) {
      card = el("figure", { id: `preview-${cam.name}`, class: "preview" },
        el("img", { alt: cam.title }), el("figcaption", {}, cam.title));
      box.append(card);
    }
    const img = card.querySelector("img");
    const wanted = cam.running && !document.hidden && previewOn();
    if (wanted && !img.dataset.live) {
      img.dataset.live = "1";
      img.onerror = () => { delete img.dataset.live; };
      img.src = `/api/cameras/${cam.name}/stream?t=${Date.now()}`;
    } else if (!wanted && img.dataset.live) {
      delete img.dataset.live;
      img.removeAttribute("src");
    }
    card.classList.toggle("offline", !cam.running);
  }
  for (const card of [...box.children]) {
    if (!cams.some((c) => `preview-${c.name}` === card.id)) card.remove();
  }
}

// The three camera views can be hidden (no streaming); the choice is remembered in this browser.
const PREVIEW_KEY = "fa_teleop_preview";
function previewOn() { return document.getElementById("preview-toggle").checked; }
try { document.getElementById("preview-toggle").checked = localStorage.getItem(PREVIEW_KEY) !== "off"; } catch (_) { /* ignore */ }
document.getElementById("preview-toggle").addEventListener("change", () => {
  try { localStorage.setItem(PREVIEW_KEY, previewOn() ? "on" : "off"); } catch (_) { /* ignore */ }
  if (lastState) renderPreviews(lastState);
});

// Control mode buttons (same transitions as the RViz FSM panel; the server re-checks them).
const MODES = [
  { action: "HOLD", label: "HOLD", state: 2 },
  { action: "HOME", label: "HOME", state: 1, confirm: "机械臂将运动到 HOME 位姿，确认周围安全？" },
  { action: "OCS2", label: "OCS2", state: 3 },
  { action: "MOVEJ", label: "MOVEJ", state: 4 },
  { action: "SWITCH_POSE", label: "切换姿态 (Home ↔ Rest)", confirm: "机械臂将在 Home 与 Rest 姿态之间运动，确认周围安全？" },
];

async function switchMode(mode) {
  if (mode.confirm && !(await confirmDialog(mode.confirm))) return;
  await action(`/api/fsm/${mode.action}`, `切换到 ${mode.label}`);
}

function renderModes(state) {
  const current = document.getElementById("fsm-current");
  current.textContent = state.fsm_name;
  const allowed = new Set(state.fsm_actions || []);
  document.getElementById("modes").replaceChildren(...MODES.map((m) => {
    const isCurrent = m.state !== undefined && m.state === state.fsm;
    const button = el("button", { class: isCurrent ? "current" : "", onclick: () => switchMode(m) }, m.label);
    button.disabled = !allowed.has(m.action);
    return button;
  }));

  const rviz = (state.units || []).find((u) => u.name === "rviz");
  const control = (state.units || []).find((u) => u.name === "control");
  const toggle = document.getElementById("rviz-toggle");
  toggle.textContent = rviz && rviz.running ? "关闭 RViz" : "打开 RViz";
  toggle.disabled = !rviz || (!rviz.running && !(control && control.running));
  toggle.title = toggle.disabled && rviz ? "先启动运控" : "";
}

// Body modes (自由/竖直/跟随…): the options come from the controller's capabilities, so the card hides until it reports.
function renderBody(state) {
  const body = state.body || { options: [], current: null };
  document.getElementById("body-section").hidden = body.options.length === 0;
  const active = body.options.find((o) => o.state === body.current);
  document.getElementById("body-current").textContent = active ? active.label : "-";
  document.getElementById("body-modes").replaceChildren(...body.options.map((o) => {
    const button = el("button", { class: o.state === body.current ? "current" : "",
                                  onclick: () => action(`/api/body/${o.action}`, `机体模式切到 ${o.label}`) }, o.label);
    return button;
  }));
}

document.getElementById("rviz-toggle").addEventListener("click", () => {
  const rviz = lastState && lastState.units.find((u) => u.name === "rviz");
  if (!rviz) return;
  // Stopping RViz never touches the controllers, so no HOLD confirmation is involved.
  if (rviz.running) guarded("/api/units/rviz/stop", "关闭 RViz");
  else action("/api/units/rviz/start", "打开 RViz");
});

async function refresh() {
  try {
    lastState = await api("/api/state");
    renderStatus(lastState);
    renderAccess(lastState);
    renderModes(lastState);
    renderBody(lastState);
    renderInference(lastState);
    renderAll(lastState);
    renderUnits(lastState);
    renderPreviews(lastState);
  } catch (err) {
    const banner = document.getElementById("banner");
    banner.textContent = `连接控制台失败：${err.message}`;
    banner.className = "banner";
  }
}

async function loadLog() {
  const name = document.getElementById("log-unit").value;
  if (!name) return;
  try {
    const data = await api(`/api/units/${name}/log?lines=300`);
    const pre = document.getElementById("log");
    const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 20;
    pre.textContent = data.lines.join("\n") || "（暂无日志）";
    if (atBottom) pre.scrollTop = pre.scrollHeight;
  } catch (err) { toast(`读取日志失败：${err.message}`, true); }
}

async function scanCameras() {
  const out = document.getElementById("scan-result");
  out.replaceChildren(el("p", {}, "扫描中 ..."));
  let result;
  try {
    result = await api("/api/cameras/scan", { method: "POST" });
  } catch (err) {
    out.replaceChildren(el("p", { class: "error-cell" }, `扫描失败：${err.message}`));
    return;
  }
  if (!result.devices.length) {
    out.replaceChildren(el("p", { class: "error-cell" }, "没有发现相机：检查连接和 udev 规则（./scripts/setup.sh host）"));
    return;
  }
  const table = el("table", { class: "units" },
    el("thead", {}, el("tr", {}, ...["型号", "序列号", "连接", "端口"].map((h) => el("th", {}, h)))),
    el("tbody", {}, ...result.devices.map((d) => el("tr", {},
      el("td", {}, d.model), el("td", { class: "mono" }, d.serial),
      el("td", {}, d.connection || "-"), el("td", {}, d.usb_port || "-")))));
  const form = el("div", { class: "assign" });
  for (const role of Object.keys(ROLE_TITLES)) {
    const serial = el("select", { "data-role": role, class: "serial" },
      el("option", { value: "" }, "不使用"),
      ...result.devices.map((d) => el("option", { value: d.serial }, `${d.serial}（${d.model}）`)));
    serial.value = result.suggestion[role] || "";
    const rotation = el("select", { "data-role": role, class: "rotation" },
      ...[0, 90, 180, 270].map((r) => el("option", { value: r }, `${r}°`)));
    rotation.value = String((result.rotation && result.rotation[role]) || 0);
    form.append(el("label", {}, `${ROLE_TITLES[role]}：`, serial, " 旋转 ", rotation));
  }
  const save = el("button", { class: "primary", onclick: () => saveAssignment(result.devices) }, "保存到配置");
  const swap = el("button", { onclick: () => {
    const l = form.querySelector('.serial[data-role="left_wrist"]');
    const r = form.querySelector('.serial[data-role="right_wrist"]');
    [l.value, r.value] = [r.value, l.value];
  } }, "对调左右腕");
  const note = result.stopped.length ? el("p", { class: "hint" }, `扫描前已停止：${result.stopped.join(", ")}`) : "";
  out.replaceChildren(table, form, el("div", {}, swap, " ", save), note);
}

async function saveAssignment(devicesFound) {
  const body = {};
  const byPort = Object.fromEntries(devicesFound.map((d) => [d.serial, d.usb_port || ""]));
  for (const role of Object.keys(ROLE_TITLES)) {
    const serial = document.querySelector(`.serial[data-role="${role}"]`).value;
    const rotation = Number(document.querySelector(`.rotation[data-role="${role}"]`).value);
    body[role] = serial ? { enable: true, serial, usb_port: byPort[serial], rotation } : { enable: false };
  }
  try {
    lastState = await api("/api/cameras/assign", { method: "POST", body });
    toast("已保存；启动相机后在上方预览确认左右是否正确");
    refresh();
  } catch (err) { toast(`保存失败：${err.message}`, true); }
}

// Everything the plain "start all" starts (on-demand RViz excluded): all running -> the button stops.
// A module on its own is restarted with the ↻ next to its status. Ask first (stopping then asks again outside HOLD).
function allUnits(state) {
  return (state.units || []).filter((u) => u.name !== "rviz");
}
function renderAll(state) {
  const units = allUnits(state);
  const running = units.length > 0 && units.every((u) => u.running);
  const toggle = document.getElementById("all-toggle");
  toggle.textContent = running ? "全部停止" : "全部启动";
  toggle.className = running ? "danger" : "primary";
}
document.getElementById("all-toggle").addEventListener("click", async () => {
  const units = allUnits(lastState);
  if (units.length && units.every((u) => u.running)) {
    const ok = await confirmDialog("全部停止：将停止所有正在运行的模块，包括运控（机械臂会抱闸）。\n"
      + "运控不在 HOLD 时，下一步还会再确认并先切到 HOLD。继续？");
    if (ok) guarded("/api/all/stop", "全部停止");
    return;
  }
  const ok = await confirmDialog("全部启动：将启动运控（会使能机械臂，机械臂上电）、PC Service、头显桥和相机。\n"
    + "请确认急停在手边，机械臂、腰、头周围无人无物。继续？");
  if (ok) action("/api/all/start", "全部启动");
});
// Inference = the controllers plus the cameras (no XR, no RViz); the policy itself runs elsewhere. One toggle.
function inferenceUnits(state) {
  return (state.units || []).filter((u) => u.name === "control" || u.is_camera);
}
function renderInference(state) {
  const units = inferenceUnits(state);
  const running = units.length > 0 && units.every((u) => u.running);
  const toggle = document.getElementById("inference-toggle");
  toggle.textContent = running ? "关闭推理" : "启动推理";
  toggle.className = running ? "danger" : "primary";
  toggle.hidden = !units.some((u) => u.name === "control");
}
document.getElementById("inference-toggle").addEventListener("click", async () => {
  const units = inferenceUnits(lastState);
  if (units.length && units.every((u) => u.running)) {
    const ok = await confirmDialog("关闭推理：将停止运控和相机（机械臂会抱闸）。\n运控不在 HOLD 时，下一步还会再确认并先切到 HOLD。继续？");
    if (ok) guarded("/api/inference/stop", "关闭推理");
    return;
  }
  const ok = await confirmDialog("启动推理：将启动运控（会使能机械臂，机械臂上电）和相机，不启动头显和 RViz。\n"
    + "请确认急停在手边，机械臂、腰、头周围无人无物。继续？");
  if (ok) action("/api/inference/start", "启动推理");
});
document.getElementById("scan").addEventListener("click", scanCameras);
document.getElementById("log-unit").addEventListener("change", loadLog);
document.addEventListener("visibilitychange", () => lastState && renderPreviews(lastState));

refresh().then(loadLog);
setInterval(refresh, 1000);
setInterval(() => { if (document.getElementById("log-follow").checked) loadLog(); }, 2000);
