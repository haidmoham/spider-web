import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { CadencePolicy } from "./cadence-policy.js";

const directory = new URL("./", import.meta.url);
const spec = JSON.parse(await readFile(new URL("walk_fast_500.json", directory), "utf8"));
const fixture = JSON.parse(await readFile(new URL("walk_fast_500.parity.json", directory), "utf8"));
assert.equal(fixture.checkpointSha256, spec.provenance.checkpointSha256);
const policy = new CadencePolicy(spec);
let maxObservationError = 0;
let maxTargetError = 0;
let maxPhaseError = 0;
for (const expected of fixture.steps) {
  const state = { time: expected.time, qpos: expected.qpos, qvel: expected.qvel };
  const observation = policy.observation(state);
  maxObservationError = Math.max(maxObservationError, ...observation.map((x, i) => Math.abs(x - expected.observation[i])));
  const targets = policy.targets(state);
  maxTargetError = Math.max(maxTargetError, ...targets.map((x, i) => Math.abs(x - expected.targets[i])));
  maxPhaseError = Math.max(maxPhaseError, Math.abs(policy.diagnostics.phaseCycles - expected.phaseCycles));
}
assert.ok(maxObservationError < 2e-5, `observation error ${maxObservationError}`);
assert.ok(maxTargetError < 2e-5, `target error ${maxTargetError}`);
assert.ok(maxPhaseError < 3e-8, `phase error ${maxPhaseError}`);
console.log(JSON.stringify({ steps: fixture.steps.length, maxObservationError, maxTargetError, maxPhaseError }));
