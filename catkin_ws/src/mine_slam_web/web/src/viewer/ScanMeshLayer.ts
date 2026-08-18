import * as THREE from 'three';
import { ColorMode, colorForPoint } from './ColorMap';
import type { ParsedCloud } from './CloudTypes';
import { buildScanMesh, type ScanMeshBuildResult } from './ScanMeshBuilder';

export interface ScanMeshDebug {
  point_count: number;
  triangle_count: number;
  sensor_count: number;
  ring_count: number;
  ring_pair_count: number;
  rejected_triangles: number;
  visible: boolean;
  protocol_version: number;
  has_lidar_id: boolean;
  has_ring: boolean;
  has_time: boolean;
  last_packet_stamp: string;
  build_reason: string;
}

export class ScanMeshLayer {
  readonly mesh: THREE.Mesh;
  private geometry = new THREE.BufferGeometry();
  private material = new THREE.MeshBasicMaterial({
    vertexColors: true,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.48,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1
  });
  private cloud: ParsedCloud | null = null;
  private colors = new Float32Array(0);
  private visible = true;
  private buildResult: ScanMeshBuildResult = {
    indices: new Uint32Array(0),
    triangleCount: 0,
    sensorCount: 0,
    ringCount: 0,
    ringPairCount: 0,
    rejectedTriangles: 0,
    reason: 'empty_cloud'
  };

  constructor() {
    this.mesh = new THREE.Mesh(this.geometry, this.material);
    this.mesh.name = 'current_scan_mesh';
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 1;
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.mesh.visible = visible;
  }

  setOpacity(opacity: number): void {
    this.material.opacity = THREE.MathUtils.clamp(opacity, 0.05, 1.0);
    this.material.needsUpdate = true;
  }

  updateCloud(
    cloud: ParsedCloud,
    mode: ColorMode,
    reflectorVisible: boolean,
    layerVisible = this.visible
  ): void {
    this.cloud = cloud;
    this.buildResult = buildScanMesh(cloud);
    this.colors = new Float32Array(cloud.pointCount * 3);

    // Release the previous frame's GPU buffers before replacing attributes.
    this.geometry.dispose();
    this.geometry.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
    this.geometry.setIndex(
      this.buildResult.indices.length > 0
        ? new THREE.BufferAttribute(this.buildResult.indices, 1)
        : null
    );
    this.geometry.setDrawRange(0, this.buildResult.indices.length);
    this.recolor(mode, reflectorVisible);

    const positionAttribute = this.geometry.getAttribute('position');
    const colorAttribute = this.geometry.getAttribute('color');
    if (positionAttribute) positionAttribute.needsUpdate = true;
    if (colorAttribute) colorAttribute.needsUpdate = true;
    if (this.geometry.index) this.geometry.index.needsUpdate = true;
    this.mesh.frustumCulled = false;
    this.setVisible(layerVisible);
  }

  recolor(mode: ColorMode, reflectorVisible: boolean): void {
    if (!this.cloud) return;
    const positions = this.cloud.positions;
    for (let i = 0; i < this.cloud.pointCount; i += 1) {
      const [r, g, b] = colorForPoint(
        {
          x: positions[i * 3],
          y: positions[i * 3 + 1],
          z: positions[i * 3 + 2],
          intensity: this.cloud.intensities[i],
          lidarId: this.cloud.lidarIds[i],
          classId: this.cloud.classIds[i],
          cloudType: this.cloud.cloudType
        },
        mode,
        reflectorVisible
      );
      this.colors[i * 3] = r;
      this.colors[i * 3 + 1] = g;
      this.colors[i * 3 + 2] = b;
    }
    const colorAttribute = this.geometry.getAttribute('color');
    if (colorAttribute) colorAttribute.needsUpdate = true;
  }

  getTriangleCount(): number {
    return this.buildResult.triangleCount;
  }

  getDebug(): ScanMeshDebug {
    return {
      point_count: this.cloud?.pointCount ?? 0,
      triangle_count: this.buildResult.triangleCount,
      sensor_count: this.buildResult.sensorCount,
      ring_count: this.buildResult.ringCount,
      ring_pair_count: this.buildResult.ringPairCount,
      rejected_triangles: this.buildResult.rejectedTriangles,
      visible: this.mesh.visible,
      protocol_version: this.cloud?.protocolVersion ?? 0,
      has_lidar_id: this.cloud?.hasLidarId ?? false,
      has_ring: this.cloud?.hasRing ?? false,
      has_time: this.cloud?.hasTime ?? false,
      last_packet_stamp: this.cloud?.stampNs.toString() ?? '0',
      build_reason: this.buildResult.reason
    };
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
  }
}
