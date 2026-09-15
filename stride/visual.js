import * as THREE from 'three';

// Visuals use the saved model's compiled geoms and sites. They never change physics.
export class StrideVisual {
  constructor(scene, payload) {
    this.group = new THREE.Group();
    scene.add(this.group);
    this.matrix = new THREE.Matrix4();
    this.lastTime = 0;
    const specs = [...payload.render.geoms.map(s => ({...s, kind: 'geom'})), ...payload.render.sites.map(s => ({...s, kind: 'site'}))];
    this.items = specs.filter(s => s.rgba[3] > 0 && s.type !== 0).map(spec => {
      const [x, y, z] = spec.size;
      let geometry;
      if (spec.type === 2 || spec.type === 4) {
        geometry = new THREE.SphereGeometry(1, 24, 16);
        geometry.scale(x, spec.type === 2 ? x : y, spec.type === 2 ? x : z);
      } else if (spec.type === 3) {
        geometry = new THREE.CapsuleGeometry(x, 2 * y, 6, 16);
        geometry.rotateX(Math.PI / 2);
      } else if (spec.type === 5) {
        geometry = new THREE.CylinderGeometry(x, x, 2 * y, 20);
        geometry.rotateX(Math.PI / 2);
      } else if (spec.type === 6) {
        geometry = new THREE.BoxGeometry(2 * x, 2 * y, 2 * z);
      } else if (spec.type === 7) {
        geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(payload.render.meshes[spec.meshId].vertices.flat(), 3));
        geometry.setIndex(payload.render.meshes[spec.meshId].indices.flat());
        geometry.computeVertexNormals();
      } else return null;
      const red = spec.rgba[0] > 0.6 && spec.rgba[1] < 0.1;
      const eye = /eye_visual/.test(spec.name);
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(...spec.rgba.slice(0, 3)),
        metalness: eye ? 0.03 : 0.65, roughness: eye ? 0.28 : 0.36,
        emissive: red ? 0xe5000a : 0, emissiveIntensity: red ? 0.7 : 0,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.group.add(mesh);
      return { spec, mesh, red, offset: new THREE.Vector2(0, -0.015), velocity: new THREE.Vector2() };
    }).filter(Boolean);
  }

  update(data) {
    const elapsed = data.time - this.lastTime;
    if (elapsed < 0) this.items.forEach(item => { item.offset.set(0, -0.015); item.velocity.set(0, 0); });
    const dt = Math.min(0.04, Math.max(0, elapsed));
    this.lastTime = data.time;
    for (const item of this.items) {
      const { spec, mesh } = item;
      const positions = spec.kind === 'site' ? data.site_xpos : data.geom_xpos;
      const rotations = spec.kind === 'site' ? data.site_xmat : data.geom_xmat;
      const p = spec.id * 3, r = spec.id * 9;
      mesh.position.set(positions[p], positions[p + 1], positions[p + 2]);
      this.matrix.set(rotations[r], rotations[r+1], rotations[r+2], 0,
        rotations[r+3], rotations[r+4], rotations[r+5], 0,
        rotations[r+6], rotations[r+7], rotations[r+8], 0, 0, 0, 0, 1);
      mesh.quaternion.setFromRotationMatrix(this.matrix);
      if (item.red) mesh.material.emissiveIntensity = 0.65 + 0.3 * Math.sin(data.time * 2.5) + 0.15 * Math.pow(Math.max(0, Math.sin(data.time * 7)), 8);
      if (/pupil_visual/.test(spec.name)) {
        const eyeName = spec.name.replace('pupil', 'eye');
        const eye = this.items.find(candidate => candidate.spec.name === eyeName);
        if (!eye) continue;
        const er = eye.spec.id * 9, ep = eye.spec.id * 3;
        // Gravity projected onto the eye's local YZ plane keeps pupils hanging down.
        const target = new THREE.Vector2(-data.site_xmat[er+7], -data.site_xmat[er+8]).multiplyScalar(0.024);
        const substeps = Math.max(1, Math.ceil(dt / 0.005));
        for (let step = 0; step < substeps; step += 1) {
          item.velocity.addScaledVector(target.clone().sub(item.offset).multiplyScalar(80).addScaledVector(item.velocity, -8), dt / substeps);
          item.offset.addScaledVector(item.velocity, dt / substeps).clampLength(0, 0.025);
        }
        const local = new THREE.Vector3(Math.sqrt(0.044 ** 2 - item.offset.lengthSq()), item.offset.x, item.offset.y);
        local.applyQuaternion(mesh.quaternion);
        mesh.position.set(data.site_xpos[ep], data.site_xpos[ep+1], data.site_xpos[ep+2]).add(local);
      }
    }
  }
}
