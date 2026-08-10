# C-1N browser artifact

This repository is the standalone WebGL renderer for **C-1N**, the longitudinal six-legged MuJoCo robot formerly referred to generically as Spider.

It runs the pinned current checkpoint and a browser port of the canonical gait controller with first-party MuJoCo WebAssembly.

## Current checkpoint

```text
C-1N // 01 · SHUFFLE
```

This checkpoint preserves the coordinated tripod gait failure. It is intentionally not presented as sustained walking.

Future public checkpoints use the grammar:

```text
C-1N // NN · CODENAME
```

The number preserves chronology. The codename records the capability or understanding gained at that boundary.

Historical and reserved names:

```text
C-1N // 00 · POSE    motor-assisted static-pose baseline; not a demonstrated standing capability
C-1N // 02 · FRAME   task-space instrumentation after bench issue #6 closes
C-1N // 03 · STAND   first support-aware stable stance
C-1N // 04 · STRIDE  first materially better walk
```

`POSE` is historical. The other names are not completed releases yet.

## Scope

The canonical robot model and controller live in
[haidmoham/spider](https://github.com/haidmoham/spider). The repository slug is legacy; the robot identity is C-1N.

This repository owns the browser rendering layer: orbit, pan, zoom, follow camera, checkpoint selection, and live runtime telemetry. It does not claim that C-1N has sustained walking.

The renderer preserves the model's visual definitions: near-black torso and legs, red feet, and an extended blue-gray paper floor.

## Run locally

Serve the directory so the browser can load modules and WebAssembly:

```sh
python -m http.server 4173 --bind 127.0.0.1
```

Then open <http://127.0.0.1:4173>.

## Provenance

`manifest.json` records the C-1N checkpoint, legacy release identifier, canonical robot commit, model digest, and runtime versions. `model/spider.xml` remains the pinned implementation filename for compatibility. See `THIRD_PARTY_NOTICES.md` for runtime licenses.
