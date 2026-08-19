import * as THREE from 'three';
import { ColorMode, colorForPoint } from './ColorMap';
import type { ParsedCloud } from './CloudTypes';
import {
  buildStableScanMesh,
  type ScanMeshBuildResult
} from './StableScanMeshBuilder';

export interface ScanMeshDebug {
  point_count: number;
  triangle_count: number;
  candidate_triangle_count: number;
  sensor_count: number;
  ring_count: number;
  ring_pair_count: number;
  rejected_triangles: number;
  visible: boolean;
  protocol_version: number;
  has_lidar_id: boolean;
  has_ring: boolean;
  has_time: boolean;
  has_azimuth: boolean;
  has_range: boolean;
  held_frame_count: number;
  frame_state: string;
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
    opacity: 0.60,
    depthTest: true,
    depthWrite: true,
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1
  });
  private cloud: ParsedCloud | null = null;
  private latestCloud: ParsedCloud | null = null;
  private colors = new Float32Array(0);
  private visible = true;
  private heldFrameCount = 0;
  private lastAppliedAtMs = 0;
  private frameState = 'empty';
  private readonly lowCoverageRatio = 0.30;
  private readonly minimumTriangleFloor = 64;
  private readonly maxHoldDurationMs = 900;
  private buildResult: ScanMeshBuildResult = emptyBuildResult();
  private latestBuildResult: ScanMeshBuildResult = emptyBuildResult();

  constructor() {
    this.mesh = new THREE.Mesh(this.geometry, this.material);
    this.mesh.name = 'current_scan_mesh';
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 1;
    (
      this.material as THREE.MeshBasicMaterial & {
        forceSinglePass: boolean;
      }
    ).forceSinglePass = true;
  }

  setVisible(visible: boolean): void {
    this.visible = visible;
    this.mesh.visible = visible;
  }

  setOpacity(opacity: number): void {
    this.material.opacity = THREE.MathUtils.clamp(opacity, 0.10, 1.0);
    this.material.needsUpdate = true;
  }

  updateCloud(
    cloud: ParsedCloud,
    mode: ColorMode,
    reflectorVisible: boolean,
    layerVisible = this.visible
  ): void {
    this.latestCloud = cloud;
    const candidate = buildStableScanMesh(cloud);
    this.latestBuildResult = candidate;

    const now = performance.now();
    const displayedTriangles = this.buildResult.triangleCount;
    const hasDisplayedMesh = this.cloud !== null && displayedTriangles > 0;
    const minimumAcceptedTriangles = Math.max(
      this.minimumTriangleFloor,
      Math.floor(displayedTriangles * this.lowCoverageRatio)
    );
    const candidateCollapsed =
      candidate.reason !== 'ok' || candidate.triangleCount < minimumAcceptedTriangles;
    const canHoldPrevious =
      hasDisplayedMesh && now - this.lastAppliedAtMs < this.maxHoldDurationMs;

    if (candidateCollapsed && canHoldPrevious) {
      this.heldFrameCount += 1;
      this.frameState = `held_low_coverage:${candidate.triangleCount}`;
      this.setVisible(layerVisible);
      return;
    }

    this.applyCloud(cloud, candidate, mode, reflectorVisible);
    this.lastAppliedAtMs = now;
    this.heldFrameCount = 0;
    this.frameState = candidate.triangleCount > 0 ? 'applied' : 'cleared';
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
    const packetCloud = this.latestCloud ?? this.cloud;
    return {
      point_count: packetCloud?.pointCount ?? 0,
      triangle_count: this.buildResult.triangleCount,
      candidate_triangle_count: this.latestBuildResult.triangleCount,
      sensor_count: this.latestBuildResult.sensorCount,
      ring_count: this.latestBuildResult.ringCount,
      ring_pair_count: this.latestBuildResult.ringPairCount,
      rejected_triangles: this.latestBuildResult.rejectedTriangles,
      visible: this.mesh.visible,
      protocol_version: packetCloud?.protocolVersion ?? 0,
      has_lidar_id: packetCloud?.hasLidarId ?? false,
      has_ring: packetCloud?.hasRing ?? false,
      has_time: packetCloud?.hasTime ?? false,
      has_azimuth: packetCloud?.hasAzimuth ?? false,
      has_range: packetCloud?.hasRange ?? false,
      held_frame_count: this.heldFrameCount,
      frame_state: this.frameState,
      last_packet_stamp: packetCloud?.stampNs.toString() ?? '0',
      build_reason: this.latestBuildResult.reason
    };
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
  }

  private applyCloud(
    cloud: ParsedCloud,
    result: ScanMeshBuildResult,
    mode: ColorMode,
    reflectorVisible: boolean
  ): void {
    const nextGeometry = new THREE.BufferGeometry();
    const nextColors = new Float32Array(cloud.pointCount * 3);
    nextGeometry.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
    nextGeometry.setAttribute('color', new THREE.BufferAttribute(nextColors, 3));
    nextGeometry.setIndex(
      result.indices.length > 0 ? new THREE.BufferAttribute(result.indices, 1) : null
    );
    nextGeometry.setDrawRange(0, result.indices.length);

    const previousGeometry = this.geometry;
    this.geometry = nextGeometry;
    this.mesh.geometry = nextGeometry;
    this.cloud = cloud;
    this.colors = nextColors;
    this.buildResult = result;
    this.recolor(mode, reflectorVisible);

    const positionAttribute = this.geometry.getAttribute('position');
    const colorAttribute = this.geometry.getAttribute('color');
    if (positionAttribute) positionAttribute.needsUpdate = true;
    if (colorAttribute) colorAttribute.needsUpdate = true;
    if (this.geometry.index) this.geometry.index.needsUpdate = true;
    this.mesh.frustumCulled = false;

    previousGeometry.dispose();
  }
}

function emptyBuildResult(): ScanMeshBuildResult {
  return {
    indices: new Uint32Array(0),
    triangleCount: 0,
    sensorCount: 0,
    ringCount: 0,
    ringPairCount: 0,
    rejectedTriangles: 0,
    reason: 'empty_cloud'
  };
}
