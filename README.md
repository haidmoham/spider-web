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
