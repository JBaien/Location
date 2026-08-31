import * as THREE from 'three';

import type { ParsedLocalTsdfMesh } from './LocalTsdfMeshClient';

export interface LocalTsdfMeshDebug {
  visible: boolean;
  revision: string;
  stamp_ns: string;
  frame_id: string;
  vertex_count: number;
  triangle_count: number;
  packet_bytes: number;
  geometry_bytes: number;
  last_operation: 'replace' | 'clear' | 'none';
}

export class LocalTsdfMeshLayer {
  readonly mesh: THREE.Mesh;
  private geometry = new THREE.BufferGeometry();
  private material = new THREE.MeshBasicMaterial({
    vertexColors: true,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 1,
    depthTest: true,
    depthWrite: true,
    polygonOffset: true,
    polygonOffsetFactor: 2,
    polygonOffsetUnits: 1
  });
  private visible = true;
  private debug: LocalTsdfMeshDebug = emptyDebug();

  constructor() {
    this.mesh = new THREE.Mesh(this.geometry, this.material);
    this.mesh.name = 'local_tsdf_mesh';
    this.mesh.frustumCulled = true;
    this.mesh.renderOrder = 0;
    (
      this.material as THREE.MeshBasicMaterial & { forceSinglePass: boolean }
    ).forceSinglePass = true;
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.mesh.visible = visible;
    this.debug.visible = visible;
  }

  updateMesh(snapshot: ParsedLocalTsdfMesh): void {
    if (snapshot.operation === 'clear') {
      this.replaceGeometry(new THREE.BufferGeometry());
      this.debug = {
        visible: this.visible,
        revision: snapshot.revision.toString(),
        stamp_ns: snapshot.stampNs.toString(),
        frame_id: snapshot.frameId,
        vertex_count: 0,
        triangle_count: 0,
        packet_bytes: snapshot.packetBytes,
        geometry_bytes: 0,
        last_operation: 'clear'
      };
      return;
    }

    const nextGeometry = new THREE.BufferGeometry();
    nextGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(snapshot.positions, 3)
    );
    nextGeometry.setAttribute(
      'color',
      new THREE.Uint8BufferAttribute(snapshot.colors, 4, true)
    );
    if (snapshot.indices) {
      nextGeometry.setIndex(new THREE.BufferAttribute(snapshot.indices, 1));
      nextGeometry.setDrawRange(0, snapshot.indices.length);
    } else {
      nextGeometry.setDrawRange(0, snapshot.vertexCount);
    }
    nextGeometry.computeBoundingBox();
    nextGeometry.computeBoundingSphere();
    this.replaceGeometry(nextGeometry);

    this.debug = {
      visible: this.visible,
      revision: snapshot.revision.toString(),
      stamp_ns: snapshot.stampNs.toString(),
      frame_id: snapshot.frameId,
      vertex_count: snapshot.vertexCount,
      triangle_count: snapshot.triangleCount,
      packet_bytes: snapshot.packetBytes,
      geometry_bytes:
        snapshot.positions.byteLength +
        snapshot.colors.byteLength +
        (snapshot.indices?.byteLength ?? 0),
      last_operation: 'replace'
    };
  }

  clear(): void {
    this.replaceGeometry(new THREE.BufferGeometry());
    this.debug = {
      ...emptyDebug(),
      visible: this.visible,
      last_operation: 'clear'
    };
  }

  getTriangleCount(): number {
    return this.debug.triangle_count;
  }

  getBoundingBox(): THREE.Box3 | null {
    const box = this.geometry.boundingBox;
    return box && !box.isEmpty() ? box.clone() : null;
  }

  getDebug(): LocalTsdfMeshDebug {
    return { ...this.debug };
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
  }

  private replaceGeometry(nextGeometry: THREE.BufferGeometry): void {
    const previousGeometry = this.geometry;
    this.geometry = nextGeometry;
    this.mesh.geometry = nextGeometry;
    this.mesh.visible = this.visible;
    previousGeometry.dispose();
  }
}

function emptyDebug(): LocalTsdfMeshDebug {
  return {
    visible: true,
    revision: '0',
    stamp_ns: '0',
    frame_id: '',
    vertex_count: 0,
    triangle_count: 0,
    packet_bytes: 0,
    geometry_bytes: 0,
    last_operation: 'none'
  };
}
