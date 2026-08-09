# spider-web

`spider-web` is the standalone WebGL renderer for the Spider MuJoCo robot.
It runs the pinned v0.1 MJCF snapshot and a browser port of the canonical gait
controller with first-party MuJoCo WebAssembly.

## Scope

The canonical robot model and controller live in
[haidmoham/spider](https://github.com/haidmoham/spider). This repository owns
the browser rendering layer: orbit, pan, zoom, follow camera, and live runtime
telemetry. It does not claim that Spider has sustained walking.

The renderer preserves the model's visual definitions: near-black torso and
legs, red feet, and an extended blue-gray paper floor.

## Run locally

Serve the directory so the browser can load modules and WebAssembly:

```sh
python -m http.server 4173 --bind 127.0.0.1
```

Then open <http://127.0.0.1:4173>.

## Provenance

`manifest.json` records the Spider commit, model digest, and runtime versions.
`model/spider.xml` is the pinned v0.1 snapshot from the canonical repository.
See `THIRD_PARTY_NOTICES.md` for runtime licenses.
