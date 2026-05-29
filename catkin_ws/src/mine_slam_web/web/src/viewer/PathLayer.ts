import * as THREE from 'three';

export class PathLayer {
  readonly line: THREE.Line;
  readonly poseMarker: THREE.Mesh;
  private geometry = new THREE.BufferGeometry();

  constructor() {
    const material = new THREE.LineBasicMaterial({ color: 0x2ee66b, linewidth: 2 });
    this.line = new THREE.Line(this.geometry, material);
    this.line.frustumCulled = false;

    this.poseMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xfff06a })
    );
  }

  setVisible(visible: boolean): void {
    this.line.visible = visible;
    this.poseMarker.visible = visible;
  }

  update(path: number[][], pose?: { valid: boolean; x: number; y: number; z: number }): void {
    const positions = new Float32Array(path.length * 3);
    for (let i = 0; i < path.length; i += 1) {
      positions[i * 3] = path[i][0];
      positions[i * 3 + 1] = path[i][1];
      positions[i * 3 + 2] = path[i][2];
    }
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const positionAttribute = this.geometry.getAttribute('position');
    if (positionAttribute) positionAttribute.needsUpdate = true;
    this.geometry.computeBoundingBox();
    this.geometry.computeBoundingSphere();
    this.line.frustumCulled = false;

    if (pose?.valid) {
      this.poseMarker.position.set(pose.x, pose.y, pose.z);
      this.poseMarker.visible = this.line.visible;
    }
  }

  getBoundingBox(): THREE.Box3 | null {
    this.geometry.computeBoundingBox();
    return this.geometry.boundingBox ? this.geometry.boundingBox.clone() : null;
  }

  getPointCount(): number {
    const positionAttribute = this.geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    return positionAttribute?.count ?? 0;
  }
}
