// Headless check of web/mcue.wasm before it is published (pages workflow):
// no imports, start-up to ENGAGED, an excessive cue is limited, E-stop parks.
//   node web/smoke.mjs
import { readFileSync } from 'node:fs';

const bytes = readFileSync(new URL('./mcue.wasm', import.meta.url));
const mod = new WebAssembly.Module(bytes);
const imports = WebAssembly.Module.imports(mod);
if (imports.length) throw new Error(`unexpected imports: ${JSON.stringify(imports)}`);
const W = new WebAssembly.Instance(mod, {}).exports;

const STATES = ['OFF', 'HOMING', 'PARKED', 'ENGAGING', 'ENGAGED', 'PARKING', 'FAULT', 'ESTOP'];
const G = 9.80665;
const snap = () => new Float64Array(W.memory.buffer, W.web_snapshot(), 68).slice();
const state = () => STATES[snap()[1]];
let failures = 0;
const check = (ok, what) => {
	console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`);
	if (!ok) failures++;
};

function run(seconds, input = () => [0, 0, G, 0, 0, 0], watch = null) {
	const n = Math.round(seconds * 100);
	for (let k = 0; k < n; k++) {
		const t = snap()[0];
		W.web_vehicle(...input(t));
		W.web_step(5);
		if (watch) watch(snap());
	}
}

function runUntil(st, timeout) {
	let t = 0;
	while (state() !== st && t < timeout) {
		run(0.01);
		t += 0.01;
	}
	return t;
}

check(W.web_init() === 1, 'init');
const geo = new Float64Array(W.memory.buffer, W.web_geometry(), 44).slice();
run(0.01);
check(W.web_command(1) === 0, 'ENABLE accepted');
check(runUntil('PARKED', 3) < 3, `homing to PARKED (${state()})`);
check(W.web_command(2) === 0, 'ENGAGE accepted');
check(runUntil('ENGAGED', 4) < 4, `engaging to ENGAGED (${state()})`);

// excessive cue: 3 g surge and sway; legs must stay inside the soft limits
const softLo = geo[40];
const softHi = geo[41];
let lo = Infinity;
let hi = -Infinity;
let limited = false;
const t0 = snap()[0];
const pulse = (t, a, b) => (t >= a && t < b ? 1 : 0);
run(4, (t) => [3 * G * pulse(t - t0, 0.5, 2.5), -3 * G * pulse(t - t0, 1.5, 3.5), G, 0, 0, 0], (s) => {
	for (let i = 0; i < 6; i++) {
		lo = Math.min(lo, s[8 + i]);
		hi = Math.max(hi, s[8 + i]);
	}
	limited = limited || s[5] === 1;
});
check(limited, 'workspace limiter engaged on the 3 g cue');
check(lo >= softLo && hi <= softHi,
	`legs inside the soft limits: ${(lo * 1e3).toFixed(1)}..${(hi * 1e3).toFixed(1)} mm ` +
	`(limits ${(softLo * 1e3).toFixed(1)}..${(softHi * 1e3).toFixed(1)})`);
check(state() === 'ENGAGED', `still ENGAGED after the excessive cue (${state()})`);

W.web_estop(1);
run(0.05);
check(state() === 'ESTOP', `E-stop -> ESTOP (${state()})`);
W.web_estop(0); // released: the state stays latched
run(0.1);
check(W.web_command(4) !== 0, 'RESET refused while still descending');
// stop, dwell, then the descent to park at the descent speed
let rs = 1;
let waited = 0;
while (rs !== 0 && waited < 20) {
	run(0.1);
	waited += 0.1;
	rs = W.web_command(4);
}
check(rs === 0, `RESET accepted once parked (after ${waited.toFixed(1)} s)`);
check(state() === 'PARKED', `PARKED after RESET (${state()})`);

if (failures) {
	console.error(`${failures} check(s) failed`);
	process.exit(1);
}
console.log('web smoke test passed');
