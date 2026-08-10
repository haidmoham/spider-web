# C-1N Browser Artifact

A standalone WebGL renderer for the C-1N MuJoCo simulation using first-party MuJoCo WebAssembly.

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
