# C-1N Browser Artifact

A standalone WebGL renderer for the C-1N MuJoCo simulation using first-party MuJoCo WebAssembly.

## Preserved checkpoint and current artifact

This repository preserves **C-1N // 01 · SHUFFLE** at the model revision in `manifest.json`. It is a historical browser experiment, not the current STAND artifact or evidence of sustained walking.

Open the [current C-1N browser artifact](https://c1n.mhaider.dev/). Its maintained browser source lives in [`haidmoham.github.io/spider`](https://github.com/haidmoham/haidmoham.github.io/tree/main/spider); the canonical Python simulation remains in [`spider`](https://github.com/haidmoham/spider). The browser implementation is separate from the Python controller, so the pinned model alone does not establish controller equivalence.

## Scope

- browser rendering for the robot model;
- orbit, pan, zoom, and follow camera controls;
- live runtime telemetry;
- a pinned model and controller revision for reproducible playback.

The canonical simulation model and controller live in [`haidmoham/spider`](https://github.com/haidmoham/spider).

## Run locally

Serve the repository so the browser can load modules and WebAssembly:

```sh
python -m http.server 4173 --bind 127.0.0.1
```

Then open <http://127.0.0.1:4173>.

`manifest.json` records the pinned model revision and runtime versions. See `THIRD_PARTY_NOTICES.md` for runtime licenses.

## live STRIDE policy

Open `stride.html` to run the accepted `walk_fast_500` mean policy in MuJoCo WASM. The historical `index.html` SHUFFLE artifact stays intact. The maintained portfolio imports the same `stride/` runtime and exported model.

The export pins the native checkpoint SHA, exact model XML, actor weights, reset state, and cadence/reference settings. Physics runs at 500 Hz; policy inference runs at 50 Hz. The browser uses deterministic mean actions, not sampled training actions. No training runs in the browser.

Run `node stride/parity.test.mjs` and `node stride/wasm.test.mjs`. The five-second native fixture agrees within 1.21e-6 qpos and 5.46e-5 qvel; reset repeats identically. These checks establish implementation parity for one trajectory, not new robustness evidence. Native acceptance covers 24 fixed flat-ground evaluations. Slip, contact fragmentation, terrain, and push recovery limits remain.

Regenerate with the canonical Python environment:

```powershell
python stride/export_policy.py --source-root C:/path/to/spider
```

This exporter runs a five-second native verification trajectory, never training. `stride/visual.js` renders saved model geometry/sites, red life lights, and cosmetic gravity-aware pupils without changing physics.
