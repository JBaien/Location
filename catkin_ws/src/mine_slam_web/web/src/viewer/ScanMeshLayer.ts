import * as THREE from 'three';
import { ColorMode, colorForPoint } from './ColorMap';
import type { CloudDeliveryMeta } from './BinaryCloudClient';
import type { ParsedCloud } from './CloudTypes';
import {
  buildStableScanMesh,
  QUAD_REJECT_REASONS,
  STRIP_BREAK_REASONS,
  ISOLATED_RUN_CAUSES,
  type IsolatedRunCauseCounts,
  type QuadRejectCounts,
  type QuadRejectSample,
  type RingDiagnostics,
  type RingPairDiagnostics,
  type ScanMeshBuildResult,
  type StripBreakCounts
} from './StableScanMeshBuilder';

export interface MeshFrameMetrics {
  packet_sequence: number;
  stamp_ns: string;
  protocol_version: number;
  packet_bytes: number;
  arrival_ms: number;
  inter_arrival_ms: number | null;
  parse_ms: number;
  dispatch_delay_ms: number;
  mesh_build_ms: number;
  mesh_decision_ms: number;
  geometry_allocation_ms: number;
  geometry_setup_ms: number;
  recolor_ms: number;
  buffer_swap_ms: number;
  mesh_apply_ms: number;
  mesh_total_ms: number;
  arrival_to_done_ms: number;
  point_count: number;
  valid_point_count: number;
  valid_range_point_count: number;
  range_validity: number;
  eligible_point_count: number;
  matched_point_count: number;
  used_point_count: number;
  eligible_point_relations: number;
  matched_point_relations: number;
  candidate_quad_count: number;
  accepted_quad_count: number;
  rejected_quad_count: number;
  candidate_triangle_count: number;
  displayed_triangle_count: number;
  match_coverage: number;
  point_coverage: number;
  quad_coverage: number;
  sensor_count: number;
  ring_count: number;
  ring_pair_count: number;
  truncated_by_max_triangles: boolean;
  reject_counts: QuadRejectCounts;
  strip_break_counts: StripBreakCounts;
  isolated_run_cause_counts: IsolatedRunCauseCounts;
  reject_samples: QuadRejectSample[];
  rings: RingDiagnostics[];
  ring_pairs: RingPairDiagnostics[];
  held: boolean;
  frame_state: string;
  build_reason: string;
}

export interface ScanMeshDebug {
  point_count: number;
  triangle_count: number;
  candidate_triangle_count: number;
  sensor_count: number;
  ring_count: number;
  ring_pair_count: number;
  rejected_triangles: number;
  candidate_quad_count: number;
  accepted_quad_count: number;
  rejected_quad_count: number;
  valid_point_count: number;
  valid_range_point_count: number;
  range_validity: number;
  eligible_point_count: number;
  matched_point_count: number;
  used_point_count: number;
  eligible_point_relations: number;
  matched_point_relations: number;
  match_coverage: number;
  point_coverage: number;
  quad_coverage: number;
  truncated_by_max_triangles: boolean;
  reject_counts: QuadRejectCounts;
  strip_break_counts: StripBreakCounts;
  isolated_run_cause_counts: IsolatedRunCauseCounts;
  reject_samples: QuadRejectSample[];
  top_reject_reason: string;
  mesh_build_ms: number;
  mesh_apply_ms: number;
  mesh_total_ms: number;
  inter_arrival_ms: number | null;
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

interface MeshApplyTimings {
  geometryAllocationMs: number;
  geometrySetupMs: number;
  recolorMs: number;
  bufferSwapMs: number;
  applyMs: number;
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
  private readonly frameHistoryLimit = 180;
  private buildResult: ScanMeshBuildResult = emptyBuildResult();
  private latestBuildResult: ScanMeshBuildResult = emptyBuildResult();
  private latestFrameMetrics: MeshFrameMetrics | null = null;
  private frameHistory: MeshFrameMetrics[] = [];

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
    layerVisible = this.visible,
    delivery?: CloudDeliveryMeta
  ): MeshFrameMetrics {
    this.latestCloud = cloud;
    const buildStartedAtMs = performance.now();
    const dispatchDelayMs = delivery
      ? Math.max(0, buildStartedAtMs - (delivery.arrivalMs + delivery.parseMs))
      : 0;
    const candidate = buildStableScanMesh(cloud);
    const buildCompletedAtMs = performance.now();
    this.latestBuildResult = candidate;

    const now = buildCompletedAtMs;
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
      const completedAtMs = performance.now();
      return this.recordFrameMetrics(
        cloud,
        candidate,
        delivery,
        dispatchDelayMs,
        buildCompletedAtMs - buildStartedAtMs,
        completedAtMs - buildCompletedAtMs,
        emptyApplyTimings(),
        completedAtMs - buildStartedAtMs,
        true,
        completedAtMs
      );
    }

    const applyStartedAtMs = performance.now();
    const applyTimings = this.applyCloud(cloud, candidate, mode, reflectorVisible);
    this.heldFrameCount = 0;
    this.frameState = candidate.triangleCount > 0 ? 'applied' : 'cleared';
    this.setVisible(layerVisible);
    const completedAtMs = performance.now();
    this.lastAppliedAtMs = completedAtMs;
    return this.recordFrameMetrics(
      cloud,
      candidate,
      delivery,
      dispatchDelayMs,
      buildCompletedAtMs - buildStartedAtMs,
      Math.max(
        0,
        completedAtMs - buildCompletedAtMs - applyTimings.applyMs
      ),
      applyTimings,
      completedAtMs - buildStartedAtMs,
      false,
      completedAtMs
    );
  }

  recolor(mode: ColorMode, reflectorVisible: boolean): void {
    if (!this.cloud) return;
    this.colorize(this.cloud, this.colors, mode, reflectorVisible);
    const colorAttribute = this.geometry.getAttribute('color');
    if (colorAttribute) colorAttribute.needsUpdate = true;
  }

  getTriangleCount(): number {
    return this.buildResult.triangleCount;
  }

  getDebug(): ScanMeshDebug {
    const packetCloud = this.latestCloud ?? this.cloud;
    const latestFrame = this.latestFrameMetrics;
    return {
      point_count: packetCloud?.pointCount ?? 0,
      triangle_count: this.buildResult.triangleCount,
      candidate_triangle_count: this.latestBuildResult.triangleCount,
      sensor_count: this.latestBuildResult.sensorCount,
      ring_count: this.latestBuildResult.ringCount,
      ring_pair_count: this.latestBuildResult.ringPairCount,
      rejected_triangles: this.latestBuildResult.rejectedTriangles,
      candidate_quad_count: this.latestBuildResult.candidateQuadCount,
      accepted_quad_count: this.latestBuildResult.acceptedQuadCount,
      rejected_quad_count: this.latestBuildResult.rejectedQuadCount,
      valid_point_count: this.latestBuildResult.validPointCount,
      valid_range_point_count: this.latestBuildResult.validRangePointCount,
      range_validity: this.latestBuildResult.rangeValidity,
      eligible_point_count: this.latestBuildResult.eligiblePointCount,
      matched_point_count: this.latestBuildResult.matchedPointCount,
      used_point_count: this.latestBuildResult.usedPointCount,
      eligible_point_relations: this.latestBuildResult.eligiblePointRelations,
      matched_point_relations: this.latestBuildResult.matchedPointRelations,
      match_coverage: this.latestBuildResult.matchCoverage,
      point_coverage: this.latestBuildResult.pointCoverage,
      quad_coverage: this.latestBuildResult.quadCoverage,
      truncated_by_max_triangles: this.latestBuildResult.truncatedByMaxTriangles,
      reject_counts: { ...this.latestBuildResult.rejectCounts },
      strip_break_counts: { ...this.latestBuildResult.stripBreakCounts },
      isolated_run_cause_counts: {
        ...this.latestBuildResult.isolatedRunCauseCounts
      },
      reject_samples: this.latestBuildResult.rejectSamples.map(copyRejectSample),
      top_reject_reason: topRejectReason(this.latestBuildResult.rejectCounts),
      mesh_build_ms: latestFrame?.mesh_build_ms ?? 0,
      mesh_apply_ms: latestFrame?.mesh_apply_ms ?? 0,
      mesh_total_ms: latestFrame?.mesh_total_ms ?? 0,
      inter_arrival_ms: latestFrame?.inter_arrival_ms ?? null,
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

  getLatestFrameMetrics(): MeshFrameMetrics | null {
    return this.latestFrameMetrics ? copyFrameMetrics(this.latestFrameMetrics) : null;
  }

  getFrameHistory(): MeshFrameMetrics[] {
    return this.frameHistory.map(copyFrameMetrics);
  }

  clearFrameHistory(): void {
    this.frameHistory = [];
  }

  dispose(): void {
    this.geometry.dispose();
    this.material.dispose();
  }

  private recordFrameMetrics(
    cloud: ParsedCloud,
    result: ScanMeshBuildResult,
    delivery: CloudDeliveryMeta | undefined,
    dispatchDelayMs: number,
    buildMs: number,
    decisionMs: number,
    applyTimings: MeshApplyTimings,
    totalMs: number,
    held: boolean,
    completedAtMs: number
  ): MeshFrameMetrics {
    const frame: MeshFrameMetrics = {
      packet_sequence: delivery?.packetSequence ?? 0,
      stamp_ns: cloud.stampNs.toString(),
      protocol_version: cloud.protocolVersion,
      packet_bytes: delivery?.packetBytes ?? 0,
      arrival_ms: delivery?.arrivalMs ?? 0,
      inter_arrival_ms: delivery?.interArrivalMs ?? null,
      parse_ms: delivery?.parseMs ?? 0,
      dispatch_delay_ms: dispatchDelayMs,
      mesh_build_ms: buildMs,
      mesh_decision_ms: decisionMs,
      geometry_allocation_ms: applyTimings.geometryAllocationMs,
      geometry_setup_ms: applyTimings.geometrySetupMs,
      recolor_ms: applyTimings.recolorMs,
      buffer_swap_ms: applyTimings.bufferSwapMs,
      mesh_apply_ms: applyTimings.applyMs,
      mesh_total_ms: totalMs,
      arrival_to_done_ms: delivery ? Math.max(0, completedAtMs - delivery.arrivalMs) : 0,
      point_count: cloud.pointCount,
      valid_point_count: result.validPointCount,
      valid_range_point_count: result.validRangePointCount,
      range_validity: result.rangeValidity,
      eligible_point_count: result.eligiblePointCount,
      matched_point_count: result.matchedPointCount,
      used_point_count: result.usedPointCount,
      eligible_point_relations: result.eligiblePointRelations,
      matched_point_relations: result.matchedPointRelations,
      candidate_quad_count: result.candidateQuadCount,
      accepted_quad_count: result.acceptedQuadCount,
      rejected_quad_count: result.rejectedQuadCount,
      candidate_triangle_count: result.triangleCount,
      displayed_triangle_count: this.buildResult.triangleCount,
      match_coverage: result.matchCoverage,
      point_coverage: result.pointCoverage,
      quad_coverage: result.quadCoverage,
      sensor_count: result.sensorCount,
      ring_count: result.ringCount,
      ring_pair_count: result.ringPairCount,
      truncated_by_max_triangles: result.truncatedByMaxTriangles,
      reject_counts: result.rejectCounts,
      strip_break_counts: result.stripBreakCounts,
      isolated_run_cause_counts: result.isolatedRunCauseCounts,
      reject_samples: result.rejectSamples,
      rings: result.rings,
      ring_pairs: result.ringPairs,
      held,
      frame_state: this.frameState,
      build_reason: result.reason
    };
    this.latestFrameMetrics = frame;
    this.frameHistory.push(frame);
    if (this.frameHistory.length > this.frameHistoryLimit) {
      this.frameHistory.splice(0, this.frameHistory.length - this.frameHistoryLimit);
    }
    return frame;
  }

  private applyCloud(
    cloud: ParsedCloud,
    result: ScanMeshBuildResult,
    mode: ColorMode,
    reflectorVisible: boolean
  ): MeshApplyTimings {
    const applyStartedAtMs = performance.now();
    const allocationStartedAtMs = applyStartedAtMs;
    const nextGeometry = new THREE.BufferGeometry();
    const nextColors = new Float32Array(cloud.pointCount * 3);
    const allocationCompletedAtMs = performance.now();

    nextGeometry.setAttribute('position', new THREE.BufferAttribute(cloud.positions, 3));
    nextGeometry.setAttribute('color', new THREE.BufferAttribute(nextColors, 3));
    nextGeometry.setIndex(
      result.indices.length > 0 ? new THREE.BufferAttribute(result.indices, 1) : null
    );
    nextGeometry.setDrawRange(0, result.indices.length);
    const setupCompletedAtMs = performance.now();

    this.colorize(cloud, nextColors, mode, reflectorVisible);
    const recolorCompletedAtMs = performance.now();

    const previousGeometry = this.geometry;
    this.geometry = nextGeometry;
    this.mesh.geometry = nextGeometry;
    this.cloud = cloud;
    this.colors = nextColors;
    this.buildResult = result;

    const positionAttribute = this.geometry.getAttribute('position');
    const colorAttribute = this.geometry.getAttribute('color');
    if (positionAttribute) positionAttribute.needsUpdate = true;
    if (colorAttribute) colorAttribute.needsUpdate = true;
    if (this.geometry.index) this.geometry.index.needsUpdate = true;
    this.mesh.frustumCulled = false;

    previousGeometry.dispose();
    const swapCompletedAtMs = performance.now();
    return {
      geometryAllocationMs: allocationCompletedAtMs - allocationStartedAtMs,
      geometrySetupMs: setupCompletedAtMs - allocationCompletedAtMs,
      recolorMs: recolorCompletedAtMs - setupCompletedAtMs,
      bufferSwapMs: swapCompletedAtMs - recolorCompletedAtMs,
      applyMs: swapCompletedAtMs - applyStartedAtMs
    };
  }

  private colorize(
    cloud: ParsedCloud,
    colors: Float32Array,
    mode: ColorMode,
    reflectorVisible: boolean
  ): void {
    const positions = cloud.positions;
    for (let i = 0; i < cloud.pointCount; i += 1) {
      const [r, g, b] = colorForPoint(
        {
          x: positions[i * 3],
          y: positions[i * 3 + 1],
          z: positions[i * 3 + 2],
          intensity: cloud.intensities[i],
          lidarId: cloud.lidarIds[i],
          classId: cloud.classIds[i],
          cloudType: cloud.cloudType
        },
        mode,
        reflectorVisible
      );
      colors[i * 3] = r;
      colors[i * 3 + 1] = g;
      colors[i * 3 + 2] = b;
    }
  }
}

function emptyBuildResult(): ScanMeshBuildResult {
  const rejectCounts = {} as QuadRejectCounts;
  for (const reason of QUAD_REJECT_REASONS) rejectCounts[reason] = 0;
  const stripBreakCounts = {} as StripBreakCounts;
  for (const reason of STRIP_BREAK_REASONS) stripBreakCounts[reason] = 0;
  const isolatedRunCauseCounts = {} as IsolatedRunCauseCounts;
  for (const cause of ISOLATED_RUN_CAUSES) isolatedRunCauseCounts[cause] = 0;
  return {
    indices: new Uint32Array(0),
    triangleCount: 0,
    sensorCount: 0,
    ringCount: 0,
    ringPairCount: 0,
    rejectedTriangles: 0,
    candidateQuadCount: 0,
    evaluatedQuadCount: 0,
    geometryValidQuadCount: 0,
    acceptedQuadCount: 0,
    rejectedQuadCount: 0,
    inputPointCount: 0,
    validPointCount: 0,
    validRangePointCount: 0,
    rangeValidity: 0,
    eligiblePointCount: 0,
    matchedPointCount: 0,
    usedPointCount: 0,
    eligiblePointRelations: 0,
    matchedPointRelations: 0,
    matchCoverage: 0,
    pointCoverage: 0,
    quadCoverage: 0,
    truncatedByMaxTriangles: false,
    rejectCounts,
    stripBreakCounts,
    isolatedRunCauseCounts,
    rejectSamples: [],
    rings: [],
    ringPairs: [],
    reason: 'empty_cloud'
  };
}

function emptyApplyTimings(): MeshApplyTimings {
  return {
    geometryAllocationMs: 0,
    geometrySetupMs: 0,
    recolorMs: 0,
    bufferSwapMs: 0,
    applyMs: 0
  };
}

function topRejectReason(counts: QuadRejectCounts): string {
  let topReason = 'none';
  let topCount = 0;
  for (const reason of QUAD_REJECT_REASONS) {
    if (counts[reason] > topCount) {
      topReason = reason;
      topCount = counts[reason];
    }
  }
  return topReason;
}

function copyRejectSample(sample: QuadRejectSample): QuadRejectSample {
  return {
    ...sample,
    points: {
      lower0: { ...sample.points.lower0 },
      upper0: { ...sample.points.upper0 },
      lower1: { ...sample.points.lower1 },
      upper1: { ...sample.points.upper1 }
    }
  };
}

function copyRingPairDiagnostics(pair: RingPairDiagnostics): RingPairDiagnostics {
  return {
    ...pair,
    isolatedRunCauseCounts: { ...pair.isolatedRunCauseCounts },
    rejectCounts: { ...pair.rejectCounts }
  };
}

function copyFrameMetrics(frame: MeshFrameMetrics): MeshFrameMetrics {
  return {
    ...frame,
    reject_counts: { ...frame.reject_counts },
    strip_break_counts: { ...frame.strip_break_counts },
    isolated_run_cause_counts: { ...frame.isolated_run_cause_counts },
    reject_samples: frame.reject_samples.map(copyRejectSample),
    rings: frame.rings.map((ring) => ({ ...ring })),
    ring_pairs: frame.ring_pairs.map(copyRingPairDiagnostics)
  };
}
