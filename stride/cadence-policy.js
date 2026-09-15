const TAU = 2 * Math.PI;

function dense(input, weight, bias) {
  return weight.map((row, output) => {
    let value = bias[output];
    for (let i = 0; i < input.length; i += 1) value += row[i] * input[i];
    return value;
  });
}

function matVecTranspose(matrix, vector) {
  return matrix[0].map((_, column) => matrix.reduce((sum, row, i) => sum + row[column] * vector[i], 0));
}

function clamp(value, low, high) { return Math.max(low, Math.min(high, value)); }

export class CadencePolicy {
  constructor(spec) {
    this.spec = spec;
    this.reset();
  }

  reset() {
    this.phaseOffsetCycles = 0;
    this.lastTime = null;
    this.currentCadenceHz = this.spec.control.cadence.base_frequency_hz;
    this.targetCadenceHz = this.currentCadenceHz;
    this.cadenceLatent = 0;
    this.previousResidual = new Array(18).fill(0);
    this.previousTarget = [...this.spec.control.neutral];
  }

  sync(time) {
    if (!Number.isFinite(time)) throw new Error("observed time must be finite");
    if (this.lastTime === null) { this.lastTime = time; return; }
    const elapsed = time - this.lastTime;
    if (elapsed < -1e-12) throw new Error("observed time moved backwards; call reset()");
    if (elapsed > 0) {
      this.phaseOffsetCycles += (this.currentCadenceHz - this.spec.control.cadence.base_frequency_hz) * elapsed;
      this.lastTime = time;
    }
  }

  phaseCycles(time) { return time * this.spec.control.cadence.base_frequency_hz + this.phaseOffsetCycles; }

  observation({ time, qpos, qvel }) {
    this.sync(time);
    if (qpos.length < 25 || qvel.length < 24) throw new Error("expected free-joint qpos/qvel plus 18 joints");
    const center = this.spec.control.bounds.map(([low, high]) => (low + high) / 2);
    const half = this.spec.control.bounds.map(([low, high]) => (high - low) / 2);
    const normalized = (values) => values.map((value, i) => (value - center[i]) / half[i]);
    const phase = ((this.phaseCycles(time) % 1) + 1) % 1;
    const cfg = this.spec.reference.config;
    const speed = cfg.stride_length_m * this.currentCadenceHz / cfg.stance_fraction;
    return [
      ...qvel.slice(0, 3), ...qvel.slice(3, 6).map((x) => x / 3), ...qpos.slice(3, 7),
      ...normalized(qpos.slice(7, 25)), ...qvel.slice(6, 24).map((x) => x / 10),
      (qpos[2] - cfg.height_m) / 0.10, ...normalized(this.previousTarget),
      speed, Math.sin(TAU * phase), Math.cos(TAU * phase),
    ].map((x) => clamp(x, -5, 5));
  }

  actor(observation) {
    const actor = this.spec.actor;
    const hidden1 = dense(observation, actor["legacy.network.0.weight"], actor["legacy.network.0.bias"]).map(Math.tanh);
    const hidden2 = dense(hidden1, actor["legacy.network.2.weight"], actor["legacy.network.2.bias"]).map(Math.tanh);
    return [
      ...dense(hidden2, actor["legacy.network.4.weight"], actor["legacy.network.4.bias"]),
      ...dense(hidden2, actor["cadence_head.weight"], actor["cadence_head.bias"]),
    ];
  }

  referencePose(time) {
    const ref = this.spec.reference;
    const cfg = ref.config;
    return ref.footCenters.flatMap((center, leg) => {
      const p = ((time * cfg.frequency_hz + cfg.offsets[leg]) % 1 + 1) % 1;
      let x;
      let z = center[2];
      if (p < cfg.stance_fraction) x = cfg.stride_length_m * (0.5 - p / cfg.stance_fraction);
      else {
        const u = (p - cfg.stance_fraction) / (1 - cfg.stance_fraction);
        const tangent = -cfg.stride_length_m * (1 - cfg.stance_fraction) / cfg.stance_fraction;
        x = (2*u**3-3*u**2+1)*(-cfg.stride_length_m/2) + (u**3-2*u**2+u)*tangent
          + (-2*u**3+3*u**2)*(cfg.stride_length_m/2) + (u**3-u**2)*tangent;
        z += cfg.clearance_m * Math.sin(Math.PI * u) ** 2;
      }
      const relative = [center[0] + x - ref.bases[leg][0], center[1] - ref.bases[leg][1], z - cfg.height_m - ref.bases[leg][2]];
      const [rx, ry, rz] = matVecTranspose(ref.rotations[leg], relative);
      const radial = Math.hypot(rx, ry);
      const [upper, lower] = ref.lengths[leg];
      const cosine = clamp((radial**2 + rz**2 - upper**2 - lower**2) / (2*upper*lower), -1, 1);
      const knee = Math.acos(cosine);
      const hip = Math.atan2(-rz, radial) - Math.atan2(lower * Math.sin(knee), upper + lower * Math.cos(knee));
      return [Math.atan2(ry, rx), hip, knee];
    });
  }

  targets(state) {
    const observation = this.observation(state);
    const latent = this.actor(observation);
    this.cadenceLatent = latent[18];
    const bounded = Math.tanh(this.cadenceLatent);
    const baseHz = this.spec.control.cadence.base_frequency_hz;
    this.targetCadenceHz = baseHz + (bounded < 0 ? 0.3 : 0.7) * bounded;
    const cadenceAlpha = 1 - Math.exp(-this.spec.control.intervalSeconds / this.spec.control.cadence.filter_time_constant_s);
    this.currentCadenceHz += cadenceAlpha * (this.targetCadenceHz - this.currentCadenceHz);
    const poseTime = state.time + this.phaseOffsetCycles / baseHz;
    const progress = clamp(state.time / this.spec.control.rampSeconds, 0, 1);
    const blend = progress * progress * (3 - 2 * progress);
    const pose = this.referencePose(poseTime);
    const base = pose.map((x, i) => this.spec.control.neutral[i] + blend * (x - this.spec.control.neutral[i]));
    const alpha = 1 - Math.exp(-this.spec.control.intervalSeconds / this.spec.control.residualFilterSeconds);
    this.previousResidual = latent.slice(0, 18).map((x, i) => {
      const desired = Math.tanh(x) * this.spec.control.residualLimitRadians[i];
      return this.previousResidual[i] + alpha * (desired - this.previousResidual[i]);
    });
    this.previousTarget = base.map((x, i) => clamp(x + this.previousResidual[i], ...this.spec.control.bounds[i]));
    return [...this.previousTarget];
  }

  get diagnostics() {
    const time = this.lastTime ?? 0;
    return { phaseCycles: this.phaseCycles(time), currentCadenceHz: this.currentCadenceHz,
      targetCadenceHz: this.targetCadenceHz, cadenceLatent: this.cadenceLatent };
  }
}
