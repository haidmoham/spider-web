import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import loadMujoco from "../vendor/mujoco/mujoco.js";
import { CadencePolicy } from "./cadence-policy.js";

const directory = new URL("./", import.meta.url);
const spec = JSON.parse(await readFile(new URL("walk_fast_500.json", directory), "utf8"));
const fixture = JSON.parse(await readFile(new URL("walk_fast_500.parity.json", directory), "utf8"));
const mujoco = await loadMujoco();
const model = mujoco.MjModel.from_xml_string(spec.modelXml);

function copyInto(destination, source) {
  for (let i = 0; i < source.length; i += 1) destination[i] = source[i];
}

function rollout() {
  const data = new mujoco.MjData(model);
  const policy = new CadencePolicy(spec);
  copyInto(data.qpos, spec.initial.qpos);
  copyInto(data.qvel, spec.initial.qvel);
  copyInto(data.ctrl, spec.initial.ctrl);
  data.time = 0;
  mujoco.mj_forward(model, data);
  let maxQposError = 0;
  let maxQvelError = 0;
  let minimumHeight = Infinity;
  const trace = [];
  for (const expected of fixture.steps) {
    const qpos = Array.from(data.qpos);
    const qvel = Array.from(data.qvel);
    maxQposError = Math.max(maxQposError, ...qpos.map((x, i) => Math.abs(x - expected.qpos[i])));
    maxQvelError = Math.max(maxQvelError, ...qvel.map((x, i) => Math.abs(x - expected.qvel[i])));
    minimumHeight = Math.min(minimumHeight, qpos[2]);
    const target = policy.targets({ time: data.time, qpos, qvel });
    copyInto(data.ctrl, target);
    for (let step = 0; step < 10; step += 1) mujoco.mj_step(model, data);
    trace.push(...qpos, ...qvel);
  }
  const result = { maxQposError, maxQvelError, minimumHeight, finalQpos: Array.from(data.qpos), trace };
  data.delete();
  return result;
}

const first = rollout();
const second = rollout();
// Native Python and WASM use the same MuJoCo model. The remaining difference
// comes from float32 PyTorch inference versus JavaScript number inference.
const trajectoryTolerance = 1e-4;
assert.ok(first.minimumHeight >= 0.25, `fell: minimum torso height ${first.minimumHeight}`);
assert.ok(first.maxQposError < trajectoryTolerance, `qpos mismatch ${first.maxQposError}`);
assert.ok(first.maxQvelError < trajectoryTolerance, `qvel mismatch ${first.maxQvelError}`);
assert.deepEqual(second.trace, first.trace, "reset rollout is not bit-repeatable");
assert.deepEqual(second.finalQpos, first.finalQpos, "reset final state is not bit-repeatable");
console.log(JSON.stringify({ steps: fixture.steps.length, trajectoryTolerance,
  maxQposError: first.maxQposError, maxQvelError: first.maxQvelError,
  minimumHeight: first.minimumHeight, resetBitRepeatable: true }));

model.delete();
