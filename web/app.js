// Browser demo: drives the mcue controller core (compiled to web/mcue.wasm) in
// real time and draws the platform. The vehicle input is generated here at
// 100 Hz, as the host would send it; everything else happens in the C code.
'use strict';

const G = 9.80665;
const STATES = ['OFF', 'HOMING', 'PARKED', 'ENGAGING', 'ENGAGED', 'PARKING', 'FAULT', 'ESTOP'];
const FAULTS = ['NONE', 'FOLLOWING', 'HARD_LIMIT', 'CURRENT', 'OVERRUN', 'HOMING', 'ESTOP', 'COMMS_LOST', 'INTERNAL'];
const CMD_STATUS = ['OK', 'not allowed in this state', 'E-stop input active', 'bad key', 'bad value'];
const CMD_NAMES = { 1: 'ENABLE', 2: 'ENGAGE', 3: 'PARK', 4: 'RESET' };
const PARAM_FORCE_GAIN = 1; // x 1000
const PARAM_TILT_RATE = 2;  // mdeg/s

// web_snapshot() layout (web/mcue_web.c)
const SN = { t: 0, state: 1, fault: 3, faultLeg: 4, wsSat: 5, wsScale: 6, comms: 7,
	legs: 8, pose: 20, cur: 38, joints: 44, f: 62, len: 68 };
const INPUT_HZ = 100;
const STEPS_PER_INPUT = 5; // cueing steps (500 Hz) per vehicle sample
const HISTORY_S = 10;

// ---- vehicle manoeuvres (tools/reference/scenarios.py) ----------------------

function smoothstep(t, t0, rise) {
	const u = Math.min(Math.max((t - t0) / rise, 0), 1);
	return u * u * u * (10 - 15 * u + 6 * u * u);
}
const pulse = (t, t0, t1, rise) => smoothstep(t, t0, rise) - smoothstep(t, t1, rise);

// Each returns specific force deviations (z without g) and body rates; the
// pitch/roll body motion terms are derivatives, taken numerically below.
const MANOEUVRES = {
	launch: { T: 12, f: (t) => ({ fx: 3 * pulse(t, 0.5, 8.5, 0.3), body: { pitch: -0.01 } }) },
	braking: { T: 9, f: (t) => ({ fx: -7 * pulse(t, 0.5, 3.35, 0.25), body: { pitch: -0.01 } }) },
	lane_change: { T: 7, f: (t) => {
		const ay = (t >= 0.5 && t < 3.5) ? 3 * Math.sin(2 * Math.PI * (t - 0.5) / 3) : 0;
		return { fy: ay, wz: ay / 20, body: { roll: -0.006 } };
	} },
	cornering: { T: 12, f: (t) => {
		const ay = 4 * pulse(t, 0.5, 7.5, 1.0);
		return { fy: ay, wz: ay / 20, body: { roll: -0.006 } };
	} },
	bump: { T: 4, f: (t) => {
		let fz = 0;
		let wy = 0;
		for (const [t0, sgn] of [[0.3, 1.0], [0.48, 0.7]]) {
			if (t >= t0) {
				const tau = t - t0;
				const h = Math.exp(-2.5 * tau) * Math.sin(2 * Math.PI * 1.5 * tau);
				fz += 5 * sgn * h;
				wy += (sgn > 0.9 ? 0.25 : -0.25) * h;
			}
		}
		return { fz, wy };
	} },
	excessive: { T: 7, f: (t) => ({
		fx: 3 * G * pulse(t, 0.5, 2.5, 0.1),
		fy: -3 * G * pulse(t, 1.5, 3.5, 0.1),
		fz: 3 * G * pulse(t, 1.0, 1.5, 0.05),
		wx: 2 * pulse(t, 4.0, 4.5, 0.05),
		wz: -3 * pulse(t, 4.5, 5.5, 0.05),
	}) },
};

// ---- WebAssembly ------------------------------------------------------------

let W;      // exports
let geo;    // geometry
const ui = {};

async function loadWasm() {
	const resp = fetch('mcue.wasm');
	let inst;
	if (WebAssembly.instantiateStreaming) {
		try {
			inst = await WebAssembly.instantiateStreaming(resp, {});
		} catch (e) {
			inst = null; // wrong MIME type on some servers: fall back
		}
	}
	if (!inst) {
		const bytes = await (await fetch('mcue.wasm')).arrayBuffer();
		inst = await WebAssembly.instantiate(bytes, {});
	}
	W = inst.instance.exports;
	if (!W.web_init()) {
		throw new Error('controller init failed');
	}
	const g = new Float64Array(W.memory.buffer, W.web_geometry(), 44);
	geo = {
		base: [], plat: [], h0: g[36], l0: g[37], lMin: g[38], lMax: g[39],
		softLo: g[40], softHi: g[41], cueLo: g[42], cueHi: g[43],
	};
	for (let i = 0; i < 6; i++) {
		geo.base.push([g[3 * i], g[3 * i + 1], g[3 * i + 2]]);
		geo.plat.push([g[18 + 3 * i], g[18 + 3 * i + 1], g[18 + 3 * i + 2]]);
	}
}

const snapshot = () => new Float64Array(W.memory.buffer, W.web_snapshot(), SN.len).slice();

// ---- simulation pacing and input ---------------------------------------------

const sim = {
	paused: false, slow: false, acc: 0, last: 0,
	keys: new Set(), ax: 0, ay: 0,              // manual drive, filtered
	man: null, manT0: 0,                         // active manoeuvre
	bumpT0: -1,
	prevBody: { pitch: 0, roll: 0 },
	estop: false, linkCut: false, autoEngage: true,
	hist: [], axis: 0,
	prevPose: null, prevVel: [0, 0], feltAcc: [0, 0],
};

function vehicleSample(t) {
	const dt = 1 / INPUT_HZ;
	// manual drive: first-order lag on the targets (a driver is not a step input)
	const axT = sim.keys.has('ArrowUp') ? 3 : sim.keys.has('ArrowDown') ? -7 : 0;
	const ayT = sim.keys.has('ArrowLeft') ? 4 : sim.keys.has('ArrowRight') ? -4 : 0;
	const a = 1 - Math.exp(-dt / 0.35);
	sim.ax += a * (axT - sim.ax);
	sim.ay += a * (ayT - sim.ay);

	let fx = sim.ax;
	let fy = sim.ay;
	let fz = 0;
	let wx = 0;
	let wy = 0;
	let wz = sim.ay / 20;
	let pitchBody = -0.01 * sim.ax;
	let rollBody = -0.006 * sim.ay;

	if (sim.man) {
		const m = MANOEUVRES[sim.man];
		const tau = t - sim.manT0;
		if (tau > m.T) {
			setManoeuvre(null);
		} else {
			const s = m.f(tau);
			fx += s.fx || 0;
			fy += s.fy || 0;
			fz += s.fz || 0;
			wx += s.wx || 0;
			wy += s.wy || 0;
			wz += s.wz || 0;
			if (s.body && s.body.pitch) pitchBody += s.body.pitch * (s.fx || 0);
			if (s.body && s.body.roll) rollBody += s.body.roll * (s.fy || 0);
		}
	}
	if (sim.bumpT0 >= 0) {
		const tau = t - sim.bumpT0;
		if (tau > MANOEUVRES.bump.T) {
			sim.bumpT0 = -1;
		} else {
			const s = MANOEUVRES.bump.f(tau);
			fz += s.fz;
			wy += s.wy;
		}
	}
	// body pitch/roll following the load transfer, as in scenarios.py (gradient)
	wy += (pitchBody - sim.prevBody.pitch) / dt;
	wx += (rollBody - sim.prevBody.roll) / dt;
	sim.prevBody = { pitch: pitchBody, roll: rollBody };
	return [fx, fy, G + fz, wx, wy, wz];
}

function advance(realDt) {
	if (sim.paused) return;
	sim.acc += Math.min(realDt, 0.1) * (sim.slow ? 0.25 : 1);
	const dt = 1 / INPUT_HZ;
	while (sim.acc >= dt) {
		sim.acc -= dt;
		const t = new Float64Array(W.memory.buffer, W.web_snapshot(), 1)[0];
		const v = vehicleSample(t);
		W.web_vehicle(...v);
		W.web_step(STEPS_PER_INPUT);
		record(snapshot());
	}
}

// felt specific force on the platform: translation (2nd difference of the true
// pose, 100 Hz) plus the tilt against gravity (small-angle body frame)
function record(s) {
	const dt = 1 / INPUT_HZ;
	const p = s.subarray(SN.pose, SN.pose + 6);
	let ax = 0;
	let ay = 0;
	if (sim.prevPose) {
		const vx = (p[0] - sim.prevPose[0]) / dt;
		const vy = (p[1] - sim.prevPose[1]) / dt;
		ax = (vx - sim.prevVel[0]) / dt;
		ay = (vy - sim.prevVel[1]) / dt;
		sim.prevVel = [vx, vy];
	}
	sim.prevPose = Array.from(p);
	const k = 0.35; // light smoothing of the double difference
	sim.feltAcc[0] += k * (ax - sim.feltAcc[0]);
	sim.feltAcc[1] += k * (ay - sim.feltAcc[1]);
	const roll = p[3];
	const pitch = p[4];
	const felt = [
		sim.feltAcc[0] - G * Math.sin(pitch),
		sim.feltAcc[1] + G * Math.cos(pitch) * Math.sin(roll),
	];
	sim.hist.push({
		t: s[SN.t], veh: [s[SN.f], s[SN.f + 1]], felt,
		legs: Array.from(s.subarray(SN.legs, SN.legs + 6)),
	});
	while (sim.hist.length && sim.hist[0].t < s[SN.t] - HISTORY_S) sim.hist.shift();
}

// ---- controls -------------------------------------------------------------------

function msg(text) {
	ui.cmdmsg.textContent = text;
}

function command(c) {
	const st = W.web_command(c);
	msg(st === 0 ? `${CMD_NAMES[c]} accepted` : `${CMD_NAMES[c]} refused: ${CMD_STATUS[st] || st}`);
	return st;
}

function setAxis(axis) {
	sim.axis = axis;
	ui.cue_title.textContent = axis ? 'Sway' : 'Surge';
	for (const o of document.querySelectorAll('[data-axis]')) {
		o.setAttribute('aria-pressed', String(Number(o.dataset.axis) === axis));
	}
}

function setManoeuvre(name) {
	if (name) setAxis(name === 'lane_change' || name === 'cornering' ? 1 : 0);
	sim.man = name;
	sim.manT0 = new Float64Array(W.memory.buffer, W.web_snapshot(), 1)[0];
	for (const b of document.querySelectorAll('[data-man]')) {
		b.classList.toggle('active', b.dataset.man === name);
	}
}

function bindControls() {
	for (const id of ['state', 'fault', 'cmdmsg', 'ws', 'pr', 'xyz', 'cur', 'time', 'gain', 'gain_v',
		'tilt', 'tilt_v', 'estop', 'link', 'pause', 'slow', 'cue_title']) {
		ui[id] = document.getElementById(id);
	}
	for (const b of document.querySelectorAll('[data-cmd]')) {
		b.addEventListener('click', () => {
			sim.autoEngage = false;
			command(Number(b.dataset.cmd));
		});
	}
	for (const b of document.querySelectorAll('[data-man]')) {
		b.addEventListener('click', () => setManoeuvre(b.dataset.man));
	}
	for (const b of document.querySelectorAll('[data-axis]')) {
		b.addEventListener('click', () => setAxis(Number(b.dataset.axis)));
	}
	const toggle = (el, fn) => el.addEventListener('click', () => {
		const on = el.getAttribute('aria-pressed') !== 'true';
		el.setAttribute('aria-pressed', String(on));
		fn(on);
	});
	toggle(ui.estop, (on) => {
		sim.estop = on;
		W.web_estop(on ? 1 : 0);
		sim.autoEngage = false;
		msg(on ? 'E-stop pressed: legs stop at the acceleration limit, then descend to park' :
			'E-stop released: RESET once parked, then ENABLE / ENGAGE');
	});
	toggle(ui.link, (on) => {
		sim.linkCut = on;
		W.web_stream(on ? 0 : 1);
		msg(on ? 'Host link cut: after the heartbeat timeout the controller washes out and parks' :
			'Host link restored');
	});
	toggle(ui.pause, (on) => { sim.paused = on; });
	toggle(ui.slow, (on) => { sim.slow = on; });

	const param = (input, out, key, fmt) => {
		input.value = W.web_get_param(key);
		out.textContent = fmt(Number(input.value));
		input.addEventListener('input', () => {
			const st = W.web_set_param(key, Number(input.value));
			out.textContent = fmt(Number(input.value));
			if (st !== 0) msg(`Parameter refused: ${CMD_STATUS[st] || st}`);
		});
	};
	param(ui.gain, ui.gain_v, PARAM_FORCE_GAIN, (v) => (v / 1000).toFixed(2));
	param(ui.tilt, ui.tilt_v, PARAM_TILT_RATE, (v) => `${(v / 1000).toFixed(2)} °/s`);

	window.addEventListener('keydown', (e) => {
		if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key) &&
			!(e.target instanceof HTMLInputElement)) {
			e.preventDefault();
			if (e.key === ' ') {
				sim.bumpT0 = new Float64Array(W.memory.buffer, W.web_snapshot(), 1)[0];
			} else {
				sim.keys.add(e.key);
			}
		}
	});
	window.addEventListener('keyup', (e) => sim.keys.delete(e.key));
	window.addEventListener('blur', () => sim.keys.clear());
}

// ---- drawing ----------------------------------------------------------------------

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const view = { az: -0.65, el: 0.42, drag: null };

function fitCanvas(c) {
	const dpr = window.devicePixelRatio || 1;
	const w = Math.round(c.clientWidth * dpr);
	const h = Math.round(c.clientWidth * dpr * (c.height / c.width));
	if (c.width !== w || c.height !== h) {
		c.width = w;
		c.height = h;
	}
	return dpr;
}

function project(p, c, scale) {
	// orbit camera around the z axis, then elevation; mild perspective
	const ca = Math.cos(view.az);
	const sa = Math.sin(view.az);
	const x1 = ca * p[0] - sa * p[1];
	const y1 = sa * p[0] + ca * p[1];
	const z = p[2] - geo.h0 * 0.5;
	const ce = Math.cos(view.el);
	const se = Math.sin(view.el);
	const depth = y1 * ce + z * se;
	const up = z * ce - y1 * se;
	const persp = 1 / (1 + depth * 0.18);
	return [c.width / 2 + x1 * scale * persp, c.height * 0.56 - up * scale * persp];
}

function poly(ctx, pts) {
	ctx.beginPath();
	pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
	ctx.closePath();
}

function hexOrder(points) {
	const cx = points.reduce((s, p) => s + p[0], 0) / points.length;
	const cy = points.reduce((s, p) => s + p[1], 0) / points.length;
	return points.slice().sort((a, b) => Math.atan2(a[1] - cy, a[0] - cx) - Math.atan2(b[1] - cy, b[0] - cx));
}

function drawPlatform(s) {
	const c = document.getElementById('platform');
	const dpr = fitCanvas(c);
	const ctx = c.getContext('2d');
	ctx.clearRect(0, 0, c.width, c.height);
	const R = Math.max(...geo.base.map((b) => Math.hypot(b[0], b[1])));
	const scale = (Math.min(c.width, c.height * 1.3) / (R * 2.9));
	const ink2 = css('--ink-2');
	const line = css('--line');

	// floor grid
	ctx.strokeStyle = line;
	ctx.lineWidth = 1 * dpr;
	const n = 6;
	for (let i = -n; i <= n; i++) {
		const a = (i / n) * R * 1.4;
		for (const [p, q] of [[[a, -R * 1.4, 0], [a, R * 1.4, 0]], [[-R * 1.4, a, 0], [R * 1.4, a, 0]]]) {
			const P = project(p, c, scale);
			const Q = project(q, c, scale);
			ctx.beginPath();
			ctx.moveTo(P[0], P[1]);
			ctx.lineTo(Q[0], Q[1]);
			ctx.stroke();
		}
	}

	const joints = [];
	for (let i = 0; i < 6; i++) {
		joints.push([s[SN.joints + 3 * i], s[SN.joints + 3 * i + 1], s[SN.joints + 3 * i + 2]]);
	}
	// base plate
	poly(ctx, hexOrder(geo.base).map((p) => project(p, c, scale)));
	ctx.fillStyle = css('--bg');
	ctx.fill();
	ctx.strokeStyle = ink2;
	ctx.lineWidth = 1.5 * dpr;
	ctx.stroke();

	// legs, coloured by how close they are to a soft limit
	for (let i = 0; i < 6; i++) {
		const l = s[SN.legs + i];
		const u = l >= geo.l0 ? (l - geo.l0) / (geo.softHi - geo.l0) : (geo.l0 - l) / (geo.l0 - geo.softLo);
		ctx.strokeStyle = u >= 1.0 ? css('--crit') : u >= 0.85 ? css('--warn') : css('--ink');
		ctx.lineWidth = 4 * dpr;
		ctx.lineCap = 'round';
		const A = project(geo.base[i], c, scale);
		const B = project(joints[i], c, scale);
		ctx.beginPath();
		ctx.moveTo(A[0], A[1]);
		ctx.lineTo(B[0], B[1]);
		ctx.stroke();
	}

	// platform (moving)
	const order = hexOrder(joints.map((j, i) => [geo.plat[i][0], geo.plat[i][1], i]));
	poly(ctx, order.map((o) => project(joints[o[2]], c, scale)));
	ctx.fillStyle = css('--accent') + '33';
	ctx.fill();
	ctx.strokeStyle = css('--accent');
	ctx.lineWidth = 2 * dpr;
	ctx.stroke();

	// forward marker on the platform (+x) and the vehicle input arrow
	const pc = [s[SN.pose], s[SN.pose + 1], s[SN.pose + 2] + geo.h0];
	const pitch = s[SN.pose + 4];
	const yaw = s[SN.pose + 5];
	const fwd = [pc[0] + Math.cos(yaw) * R * 0.45, pc[1] + Math.sin(yaw) * R * 0.45,
		pc[2] - Math.sin(pitch) * R * 0.45];
	arrow(ctx, project(pc, c, scale), project(fwd, c, scale), css('--accent'), 2 * dpr, 'front');
	const fx = s[SN.f];
	const fy = s[SN.f + 1];
	if (Math.hypot(fx, fy) > 0.15) {
		const k = (R * 0.9) / (3 * G);
		const tip = [pc[0] + fx * k * 3, pc[1] + fy * k * 3, pc[2] + 0.02];
		arrow(ctx, project(pc, c, scale), project(tip, c, scale), css('--muted'), 3 * dpr,
			`${Math.hypot(fx, fy).toFixed(1)} m/s²`);
	}
}

function arrow(ctx, A, B, color, w, label) {
	const ang = Math.atan2(B[1] - A[1], B[0] - A[0]);
	const h = 10 * (w / 2);
	ctx.strokeStyle = color;
	ctx.fillStyle = color;
	ctx.lineWidth = w;
	ctx.beginPath();
	ctx.moveTo(A[0], A[1]);
	ctx.lineTo(B[0], B[1]);
	ctx.stroke();
	ctx.beginPath();
	ctx.moveTo(B[0], B[1]);
	ctx.lineTo(B[0] - h * Math.cos(ang - 0.4), B[1] - h * Math.sin(ang - 0.4));
	ctx.lineTo(B[0] - h * Math.cos(ang + 0.4), B[1] - h * Math.sin(ang + 0.4));
	ctx.closePath();
	ctx.fill();
	if (label) {
		ctx.font = `${11 * (window.devicePixelRatio || 1)}px system-ui, sans-serif`;
		ctx.fillStyle = css('--ink-2');
		ctx.fillText(label, B[0] + 6, B[1] - 6);
	}
}

function chart(id, series, yMin, yMax, unit, bands) {
	const c = document.getElementById(id);
	const dpr = fitCanvas(c);
	const ctx = c.getContext('2d');
	ctx.clearRect(0, 0, c.width, c.height);
	const padL = 52 * dpr;
	const padR = 120 * dpr;
	const padT = 8 * dpr;
	const padB = 20 * dpr;
	const W0 = c.width - padL - padR;
	const H0 = c.height - padT - padB;
	const tEnd = sim.hist.length ? sim.hist[sim.hist.length - 1].t : 0;
	const X = (t) => padL + ((t - (tEnd - HISTORY_S)) / HISTORY_S) * W0;
	const Y = (v) => padT + (1 - (v - yMin) / (yMax - yMin)) * H0;
	ctx.font = `${11 * dpr}px system-ui, sans-serif`;

	// grid and axis labels
	ctx.strokeStyle = css('--line');
	ctx.fillStyle = css('--muted');
	ctx.lineWidth = 1 * dpr;
	const ticks = 4;
	for (let i = 0; i <= ticks; i++) {
		const v = yMin + ((yMax - yMin) * i) / ticks;
		ctx.beginPath();
		ctx.moveTo(padL, Y(v));
		ctx.lineTo(padL + W0, Y(v));
		ctx.stroke();
		ctx.fillText(`${Math.abs(v) < 1e-9 ? 0 : +v.toFixed(1)}`, 8 * dpr, Y(v) + 4 * dpr);
	}
	ctx.fillText(unit, 8 * dpr, padT + H0 + 16 * dpr);
	ctx.fillText(`last ${HISTORY_S} s`, padL + W0 - 50 * dpr, padT + H0 + 16 * dpr);

	for (const b of bands || []) {
		ctx.strokeStyle = b.color;
		ctx.setLineDash([5 * dpr, 4 * dpr]);
		ctx.beginPath();
		ctx.moveTo(padL, Y(b.v));
		ctx.lineTo(padL + W0, Y(b.v));
		ctx.stroke();
		ctx.setLineDash([]);
		ctx.fillStyle = css('--ink-2');
		ctx.fillText(b.label, padL + W0 + 6 * dpr, Y(b.v) + (b.above ? -4 : 12) * dpr);
	}

	ctx.save();
	ctx.beginPath();
	ctx.rect(padL, padT, W0, H0);
	ctx.clip();
	for (const s of series) {
		ctx.strokeStyle = s.color;
		ctx.lineWidth = (s.width || 2) * dpr;
		ctx.setLineDash(s.dash ? [6 * dpr, 4 * dpr] : []);
		ctx.beginPath();
		sim.hist.forEach((h, i) => {
			const v = Math.min(Math.max(s.get(h), yMin - 1), yMax + 1);
			if (i) ctx.lineTo(X(h.t), Y(v));
			else ctx.moveTo(X(h.t), Y(v));
		});
		ctx.stroke();
	}
	ctx.restore();
	ctx.setLineDash([]);

	// legend on the right, direct-labelled at the last value
	let ly = padT + (bands && bands.length ? H0 / 2 : 12 * dpr);
	for (const s of series.filter((x) => x.label)) {
		ctx.strokeStyle = s.color;
		ctx.lineWidth = 2 * dpr;
		ctx.setLineDash(s.dash ? [6 * dpr, 4 * dpr] : []);
		ctx.beginPath();
		ctx.moveTo(padL + W0 + 6 * dpr, ly - 4 * dpr);
		ctx.lineTo(padL + W0 + 22 * dpr, ly - 4 * dpr);
		ctx.stroke();
		ctx.setLineDash([]);
		ctx.fillStyle = css('--ink');
		ctx.fillText(s.label, padL + W0 + 26 * dpr, ly);
		ly += 16 * dpr;
	}
}

function drawCharts() {
	const ax = sim.axis;
	const vehMax = Math.max(4, ...sim.hist.map((h) => Math.abs(h.veh[ax])));
	const lim = Math.min(Math.ceil(vehMax + 1), 32);
	chart('chart_surge', [
		{ get: (h) => h.veh[ax], color: css('--muted'), dash: true, label: 'vehicle' },
		{ get: (h) => h.felt[ax], color: css('--accent'), label: 'platform (felt)' },
	], -lim, lim, 'm/s²');

	const mm = (l) => (l - geo.l0) * 1000;
	const legs = [];
	for (let i = 0; i < 6; i++) {
		legs.push({ get: (h) => mm(h.legs[i]), color: css('--ink-2'), width: 1.5, label: i ? '' : 'legs 1–6' });
	}
	chart('chart_legs', legs, mm(geo.lMin) - 10, mm(geo.lMax) + 10, 'mm from l₀', [
		{ v: mm(geo.softHi), color: css('--warn'), label: 'soft limit' },
		{ v: mm(geo.softLo), color: css('--warn'), label: 'soft limit', above: true },
		{ v: mm(geo.lMax), color: css('--crit'), label: 'end stop', above: true },
		{ v: mm(geo.lMin), color: css('--crit'), label: 'end stop' },
	]);
}

function updatePanel(s) {
	const st = STATES[s[SN.state]] || '?';
	ui.state.textContent = st;
	ui.state.className = 'badge' + (st === 'ENGAGED' ? ' engaged' : (st === 'FAULT' || st === 'ESTOP') ? ' crit' :
		(st === 'HOMING' || st === 'ENGAGING' || st === 'PARKING') ? ' busy' : '');
	const f = FAULTS[s[SN.fault]];
	ui.fault.textContent = (f && f !== 'NONE') ? `${f}${s[SN.faultLeg] >= 0 ? `, leg ${s[SN.faultLeg] + 1}` : ''}` :
		(s[SN.comms] ? 'host link lost' : '');
	ui.ws.textContent = s[SN.wsSat] ? `${s[SN.wsScale].toFixed(2)} (limited)` : s[SN.wsScale].toFixed(2);
	const deg = (r) => (r * 180 / Math.PI).toFixed(1);
	ui.pr.textContent = `${deg(s[SN.pose + 4])}° / ${deg(s[SN.pose + 3])}°`;
	const mm = (m) => (m * 1000).toFixed(0);
	ui.xyz.textContent = `${mm(s[SN.pose])} / ${mm(s[SN.pose + 1])} / ${mm(s[SN.pose + 2])} mm`;
	ui.cur.textContent = `${Math.max(...s.subarray(SN.cur, SN.cur + 6).map(Math.abs)).toFixed(1)} A`;
	ui.time.textContent = `${s[SN.t].toFixed(1)} s`;

	// start-up: ENABLE (homing), then ENGAGE once parked
	if (sim.autoEngage && st === 'PARKED') {
		sim.autoEngage = false;
		command(2);
		msg('Engaging. Then pick a manoeuvre or drive with the arrow keys.');
	}
}

function frame(now) {
	const dt = sim.last ? (now - sim.last) / 1000 : 0;
	sim.last = now;
	advance(dt);
	const s = snapshot();
	drawPlatform(s);
	drawCharts();
	updatePanel(s);
	requestAnimationFrame(frame);
}

function bindOrbit() {
	const c = document.getElementById('platform');
	c.addEventListener('pointerdown', (e) => {
		view.drag = [e.clientX, e.clientY, view.az, view.el];
		c.setPointerCapture(e.pointerId);
	});
	c.addEventListener('pointermove', (e) => {
		if (!view.drag) return;
		view.az = view.drag[2] - (e.clientX - view.drag[0]) * 0.006;
		view.el = Math.min(1.3, Math.max(0.05, view.drag[3] + (e.clientY - view.drag[1]) * 0.004));
	});
	c.addEventListener('pointerup', () => { view.drag = null; });
}

(async () => {
	try {
		await loadWasm();
	} catch (e) {
		document.querySelector('header p').textContent = `Could not load the WebAssembly module: ${e.message}`;
		return;
	}
	bindControls();
	bindOrbit();
	W.web_step(STEPS_PER_INPUT); // one input period first, so the controller has read the encoders
	command(1);
	msg('Homing the legs, then engaging…');
	requestAnimationFrame(frame);
})();
