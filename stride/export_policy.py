"""Export the accepted walk_fast_500 mean policy for a dependency-free JS runtime."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch


DEFAULT_SOURCE_ROOT = Path(__file__).resolve().parents[3] / "robotics" / "spider"
SOURCE_ROOT = (Path(sys.argv[sys.argv.index("--source-root") + 1]).resolve()
               if "--source-root" in sys.argv else DEFAULT_SOURCE_ROOT)
CHECKPOINT = SOURCE_ROOT / "artifacts" / "walk_fast_500" / "walk_fast_500.pt"
OUTPUT = Path(__file__).with_name("walk_fast_500.json")
FIXTURE = Path(__file__).with_name("walk_fast_500.parity.json")
sys.path.insert(0, str(SOURCE_ROOT))

from spider import simulation  # noqa: E402
from spider.cadence_action_training import CadenceResidualPolicy  # noqa: E402
from spider.chassis_candidate import reset_candidate  # noqa: E402
from spider.reference_training import effective_control_bounds  # noqa: E402


def floats(value):
    return np.asarray(value).tolist()


def render_data(model: mujoco.MjModel) -> dict:
    meshes = []
    for mesh_id in range(model.nmesh):
        mesh = model.mesh(mesh_id)
        vertex_start, face_start = int(model.mesh_vertadr[mesh_id]), int(model.mesh_faceadr[mesh_id])
        meshes.append({
            "id": mesh_id, "name": mesh.name,
            "vertices": floats(model.mesh_vert[vertex_start:vertex_start + int(model.mesh_vertnum[mesh_id])]),
            "indices": floats(model.mesh_face[face_start:face_start + int(model.mesh_facenum[mesh_id])]),
        })

    def object_record(kind: str, object_id: int) -> dict:
        obj = getattr(model, kind)(object_id)
        record = {
            "id": object_id, "name": obj.name, "type": int(np.asarray(obj.type).item()),
            "size": floats(obj.size), "rgba": floats(obj.rgba),
            "pos": floats(obj.pos), "quat": floats(obj.quat),
            "bodyId": int(np.asarray(obj.bodyid).item()),
        }
        if kind == "geom":
            geom_type = int(np.asarray(obj.type).item())
            record["meshId"] = int(np.asarray(obj.dataid).item()) if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH) else -1
        return record

    return {
        "geoms": [object_record("geom", i) for i in range(model.ngeom)],
        "sites": [object_record("site", i) for i in range(model.nsite)],
        "meshes": meshes,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE_ROOT,
                        help="canonical spider repository (default: sibling robotics/spider)")
    parser.add_argument("--checkpoint", type=Path,
                        help="checkpoint path (default: <source-root>/artifacts/walk_fast_500/walk_fast_500.pt)")
    args = parser.parse_args()
    source_root = args.source_root.resolve()
    checkpoint = (args.checkpoint or source_root / "artifacts" / "walk_fast_500" / "walk_fast_500.pt").resolve()
    checkpoint_bytes = checkpoint.read_bytes()
    payload = torch.load(checkpoint, map_location="cpu", weights_only=True)
    model = mujoco.MjModel.from_xml_string(payload["model_xml"])
    policy = CadenceResidualPolicy(checkpoint, model, sampled=False)
    data = mujoco.MjData(model)
    reset_candidate(model, data)

    actor_keys = (
        "legacy.network.0.weight", "legacy.network.0.bias",
        "legacy.network.2.weight", "legacy.network.2.bias",
        "legacy.network.4.weight", "legacy.network.4.bias",
        "cadence_head.weight", "cadence_head.bias",
    )
    export = {
        "schemaVersion": 1,
        "provenance": {
            "checkpoint": "artifacts/walk_fast_500/walk_fast_500.pt",
            "checkpointSha256": hashlib.sha256(checkpoint_bytes).hexdigest(),
            "modelXmlSha256": hashlib.sha256(payload["model_xml"].encode()).hexdigest(),
            "policyKind": payload["policy_kind"], "updates": int(payload["updates"]),
        },
        "modelXml": payload["model_xml"],
        "checkpointSettings": {
            "settings": payload["settings"], "ppo": payload["ppo"],
            "reward": payload["reward"], "executionDevice": payload["execution_device"],
        },
        "actor": {key: floats(payload["actor"][key]) for key in actor_keys},
        "control": {
            "intervalSeconds": float(payload["control_interval_s"]),
            "rampSeconds": float(payload["ramp_duration_s"]),
            "residualFilterSeconds": float(payload["filter_time_constant_s"]),
            "cadence": payload["cadence_config"],
            "bounds": floats(effective_control_bounds(model)),
            "neutral": floats(payload["neutral_targets"]),
            "residualLimitRadians": floats(payload["residual_limit_rad"]),
        },
        "reference": {
            "config": payload["reference_config"],
            "speedMetersPerSecond": float(payload["reference_speed_mps"]),
            "footCenters": floats(payload["reference_foot_centers"]),
            "bases": floats(policy.reference._bases),
            "rotations": floats(policy.reference._rotations),
            "lengths": floats(policy.reference._lengths),
            "limits": floats(policy.reference._limits),
        },
        "initial": {"qpos": floats(data.qpos), "qvel": floats(data.qvel), "ctrl": floats(data.ctrl)},
        "render": render_data(model),
    }
    OUTPUT.write_text(json.dumps(export, separators=(",", ":")), encoding="utf-8")

    steps = []
    policy.reset()
    for _ in range(int(payload["settings"]["horizon"])):
        observed = simulation.measured_state(model, data)
        observation = policy.controller.observation(observed).numpy()
        target = policy.targets(observed)
        steps.append({
            "time": float(observed.time), "qpos": floats(data.qpos), "qvel": floats(data.qvel),
            "observation": floats(observation), "targets": floats(target),
            "phaseCycles": policy.diagnostics["phase_cycles"],
            "cadenceHz": policy.diagnostics["current_cadence_hz"],
        })
        for _ in range(int(payload["settings"]["physics_steps"])):
            simulation.step(model, data, target.tolist())
    summary = {
        "durationSeconds": float(data.time), "samples": len(steps),
        "finalQpos": floats(data.qpos), "finalQvel": floats(data.qvel),
        "forwardDisplacementMeters": float(data.qpos[0] - export["initial"]["qpos"][0]),
    }
    FIXTURE.write_text(json.dumps({"checkpointSha256": export["provenance"]["checkpointSha256"],
                                   "summary": summary, "steps": steps}, separators=(",", ":")),
                       encoding="utf-8")


if __name__ == "__main__":
    main()
