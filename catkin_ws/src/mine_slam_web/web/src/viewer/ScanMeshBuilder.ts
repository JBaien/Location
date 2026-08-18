import type { ParsedCloud } from './CloudTypes';

export interface ScanMeshBuildOptions {
  maxTriangles: number;
  maxRingGap: number;
  timeMatchFactor: number;
  timeGapFactor: number;
  maxIndexGap: number;
  maxEdgeLengthM: number;
  baseEdgeLengthM: number;
  edgeLengthPerRange: number;
  maxRangeJumpM: number;
  baseRangeJumpM: number;
  rangeJumpRatio: number;
  minTriangleAreaM2: number;
}

export interface ScanMeshBuildResult {
  indices: Uint32Array;
  triangleCount: number;
  sensorCount: number;
  ringCount: number;
  ringPairCount: number;
  rejectedTriangles: number;
  reason: 'ok' | 'empty_cloud' | 'missing_lidar_id' | 'missing_ring';
}

export const DEFAULT_SCAN_MESH_OPTIONS: ScanMeshBuildOptions = {
  maxTriangles: 300000,
  maxRingGap: 1,
  timeMatchFactor: 3.5,
  timeGapFactor: 12,
  maxIndexGap: 4,
  maxEdgeLengthM: 1.0,
  baseEdgeLengthM: 0.28,
  edgeLengthPerRange: 0.075,
  maxRangeJumpM: 0.75,
  baseRangeJumpM: 0.25,
  rangeJumpRatio: 0.12,
  minTriangleAreaM2: 0.00002
};

interface RingSequence {
  ring: number;
  indices: number[];
  timeUsable: boolean;
  timeStep: number;
}

interface RingMatch {
  aPoint: number;
  bPoint: number;
  aOrder: number;
  bOrder: number;
}

export function buildScanMesh(
  cloud: ParsedCloud,
  options: ScanMeshBuildOptions = DEFAULT_SCAN_MESH_OPTIONS
): ScanMeshBuildResult {
  if (cloud.pointCount < 4) {
    return emptyResult('empty_cloud');
  }
  if (!cloud.hasLidarId) {
    return emptyResult('missing_lidar_id');
  }
  if (!cloud.hasRing) {
    return emptyResult('missing_ring');
  }

  const sensors = new Map<number, Map<number, number[]>>();
  const positions = cloud.positions;
  for (let i = 0; i < cloud.pointCount; i += 1) {
    const offset = i * 3;
    if (
      !Number.isFinite(positions[offset]) ||
      !Number.isFinite(positions[offset + 1]) ||
      !Number.isFinite(positions[offset + 2])
    ) {
      continue;
    }

    const lidarId = cloud.hasLidarId ? cloud.lidarIds[i] : 0;
    const ring = cloud.rings[i];
    let rings = sensors.get(lidarId);
    if (!rings) {
      rings = new Map<number, number[]>();
      sensors.set(lidarId, rings);
    }
    let sequence = rings.get(ring);
    if (!sequence) {
      sequence = [];
      rings.set(ring, sequence);
    }
    sequence.push(i);
  }

  const maxTriangles = Math.max(0, Math.floor(options.maxTriangles));
  const output = new Uint32Array(maxTriangles * 3);
  const ranges = computeRanges(cloud.positions, cloud.pointCount);
  let outputCount = 0;
  let rejectedTriangles = 0;
  let ringCount = 0;
  let ringPairCount = 0;

  const appendTriangle = (a: number, b: number, c: number): boolean => {
    if (outputCount >= output.length) return false;
    if (!isTriangleValid(a, b, c, cloud.positions, ranges, options)) {
      rejectedTriangles += 1;
      return true;
    }
    output[outputCount] = a;
    output[outputCount + 1] = b;
    output[outputCount + 2] = c;
    outputCount += 3;
    return outputCount < output.length;
  };

  outer: for (const rings of sensors.values()) {
    const sequences = Array.from(rings.entries())
      .map(([ring, indices]) => prepareRingSequence(ring, indices, cloud))
      .filter((sequence) => sequence.indices.length >= 2)
      .sort((a, b) => a.ring - b.ring);
    ringCount += sequences.length;

    for (let i = 0; i + 1 < sequences.length; i += 1) {
      const a = sequences[i];
      const b = sequences[i + 1];
      if (b.ring - a.ring > options.maxRingGap) continue;

      const matches = matchRingSequences(a, b, cloud, options);
      if (matches.length < 2) continue;
      ringPairCount += 1;

      for (let j = 1; j < matches.length; j += 1) {
        const previous = matches[j - 1];
        const current = matches[j];
        if (!matchesAreAdjacent(previous, current, a, b, cloud, options)) {
          continue;
        }

        const a0 = previous.aPoint;
        const a1 = current.aPoint;
        const b0 = previous.bPoint;
        const b1 = current.bPoint;
        if (a0 === a1 || b0 === b1 || a0 === b0 || a1 === b1) continue;

        const diagonalA1B0 = squaredDistance(a1, b0, positions);
        const diagonalA0B1 = squaredDistance(a0, b1, positions);
        if (diagonalA1B0 <= diagonalA0B1) {
          if (!appendTriangle(a0, b0, a1)) break outer;
          if (!appendTriangle(a1, b0, b1)) break outer;
        } else {
          if (!appendTriangle(a0, b0, b1)) break outer;
          if (!appendTriangle(a0, b1, a1)) break outer;
        }
      }
    }
  }

  return {
    indices: output.slice(0, outputCount),
    triangleCount: outputCount / 3,
    sensorCount: sensors.size,
    ringCount,
    ringPairCount,
    rejectedTriangles,
    reason: 'ok'
  };
}

function emptyResult(
  reason: 'empty_cloud' | 'missing_lidar_id' | 'missing_ring'
): ScanMeshBuildResult {
  return {
    indices: new Uint32Array(0),
    triangleCount: 0,
    sensorCount: 0,
    ringCount: 0,
    ringPairCount: 0,
    rejectedTriangles: 0,
    reason
  };
}

function prepareRingSequence(ring: number, sourceIndices: number[], cloud: ParsedCloud): RingSequence {
  const indices = sourceIndices;
  if (!cloud.hasTime || indices.length < 3) {
    return { ring, indices, timeUsable: false, timeStep: 0 };
  }

  let finiteCount = 0;
  let minimum = Number.POSITIVE_INFINITY;
  let maximum = Number.NEGATIVE_INFINITY;
  let inversions = 0;
  let previous = Number.NEGATIVE_INFINITY;
  for (const pointIndex of indices) {
    const value = cloud.times[pointIndex];
    if (!Number.isFinite(value)) continue;
    finiteCount += 1;
    minimum = Math.min(minimum, value);
    maximum = Math.max(maximum, value);
    if (value < previous) inversions += 1;
    previous = value;
  }

  const timeUsable =
    finiteCount >= Math.max(3, Math.floor(indices.length * 0.8)) &&
    maximum - minimum > Number.EPSILON;
  if (!timeUsable) {
    return { ring, indices, timeUsable: false, timeStep: 0 };
  }

  if (inversions > 0) {
    indices.sort((left, right) => comparePointTimes(left, right, cloud.times));
  }
  const timeStep = estimateTimeStep(indices, cloud.times);
  return {
    ring,
    indices,
    timeUsable: Number.isFinite(timeStep) && timeStep > 0,
    timeStep
  };
}

function comparePointTimes(left: number, right: number, times: Float32Array): number {
  const leftTime = times[left];
  const rightTime = times[right];
  if (!Number.isFinite(leftTime) && !Number.isFinite(rightTime)) return left - right;
  if (!Number.isFinite(leftTime)) return 1;
  if (!Number.isFinite(rightTime)) return -1;
  return leftTime === rightTime ? left - right : leftTime - rightTime;
}

function estimateTimeStep(indices: number[], times: Float32Array): number {
  const deltas: number[] = [];
  const sampleCount = Math.min(256, indices.length - 1);
  for (let sample = 1; sample <= sampleCount; sample += 1) {
    const order = Math.max(1, Math.floor((sample * (indices.length - 1)) / sampleCount));
    const current = times[indices[order]];
    const previous = times[indices[order - 1]];
    const delta = current - previous;
    if (Number.isFinite(delta) && delta > 0) deltas.push(delta);
  }
  if (deltas.length === 0) return 0;
  deltas.sort((a, b) => a - b);
  return deltas[Math.floor(deltas.length / 2)];
}

function matchRingSequences(
  a: RingSequence,
  b: RingSequence,
  cloud: ParsedCloud,
  options: ScanMeshBuildOptions
): RingMatch[] {
  if (a.timeUsable && b.timeUsable) {
    const matches = matchByTime(a, b, cloud.times, options);
    if (matches.length >= 2) return matches;
  }
  return matchByNormalizedOrder(a.indices, b.indices);
}

function matchByTime(
  a: RingSequence,
  b: RingSequence,
  times: Float32Array,
  options: ScanMeshBuildOptions
): RingMatch[] {
  const tolerance = Math.max(a.timeStep, b.timeStep) * options.timeMatchFactor;
  if (!Number.isFinite(tolerance) || tolerance <= 0) return [];

  const matches: RingMatch[] = [];
  let bOrder = 0;
  let previousBOrder = -1;
  for (let aOrder = 0; aOrder < a.indices.length; aOrder += 1) {
    const aPoint = a.indices[aOrder];
    const aTime = times[aPoint];
    if (!Number.isFinite(aTime)) continue;

    while (bOrder + 1 < b.indices.length) {
      const currentTime = times[b.indices[bOrder]];
      const nextTime = times[b.indices[bOrder + 1]];
      if (!Number.isFinite(currentTime)) {
        bOrder += 1;
        continue;
      }
      if (!Number.isFinite(nextTime)) break;
      if (Math.abs(nextTime - aTime) <= Math.abs(currentTime - aTime)) {
        bOrder += 1;
      } else {
        break;
      }
    }

    const bPoint = b.indices[bOrder];
    const bTime = times[bPoint];
    if (!Number.isFinite(bTime) || Math.abs(bTime - aTime) > tolerance) continue;
    if (bOrder === previousBOrder) continue;
    matches.push({ aPoint, bPoint, aOrder, bOrder });
    previousBOrder = bOrder;
  }
  return matches;
}

function matchByNormalizedOrder(a: number[], b: number[]): RingMatch[] {
  const count = Math.min(a.length, b.length);
  if (count < 2) return [];

  const matches: RingMatch[] = [];
  let previousAOrder = -1;
  let previousBOrder = -1;
  for (let sample = 0; sample < count; sample += 1) {
    const aOrder = Math.round((sample * (a.length - 1)) / (count - 1));
    const bOrder = Math.round((sample * (b.length - 1)) / (count - 1));
    if (aOrder === previousAOrder || bOrder === previousBOrder) continue;
    matches.push({
      aPoint: a[aOrder],
      bPoint: b[bOrder],
      aOrder,
      bOrder
    });
    previousAOrder = aOrder;
    previousBOrder = bOrder;
  }
  return matches;
}

function matchesAreAdjacent(
  previous: RingMatch,
  current: RingMatch,
  a: RingSequence,
  b: RingSequence,
  cloud: ParsedCloud,
  options: ScanMeshBuildOptions
): boolean {
  if (current.aOrder <= previous.aOrder || current.bOrder <= previous.bOrder) return false;

  if (a.timeUsable && b.timeUsable) {
    const aGap = cloud.times[current.aPoint] - cloud.times[previous.aPoint];
    const bGap = cloud.times[current.bPoint] - cloud.times[previous.bPoint];
    const maximumGap = Math.max(a.timeStep, b.timeStep) * options.timeGapFactor;
    return (
      Number.isFinite(aGap) &&
      Number.isFinite(bGap) &&
      aGap > 0 &&
      bGap > 0 &&
      aGap <= maximumGap &&
      bGap <= maximumGap
    );
  }

  return (
    current.aOrder - previous.aOrder <= options.maxIndexGap &&
    current.bOrder - previous.bOrder <= options.maxIndexGap
  );
}

function computeRanges(positions: Float32Array, pointCount: number): Float32Array {
  const ranges = new Float32Array(pointCount);
  for (let i = 0; i < pointCount; i += 1) {
    const offset = i * 3;
    ranges[i] = Math.hypot(positions[offset], positions[offset + 1], positions[offset + 2]);
  }
  return ranges;
}

function isTriangleValid(
  a: number,
  b: number,
  c: number,
  positions: Float32Array,
  ranges: Float32Array,
  options: ScanMeshBuildOptions
): boolean {
  if (a === b || b === c || a === c) return false;

  const ab = squaredDistance(a, b, positions);
  const bc = squaredDistance(b, c, positions);
  const ca = squaredDistance(c, a, positions);
  if (!Number.isFinite(ab) || !Number.isFinite(bc) || !Number.isFinite(ca)) return false;

  const minimumRange = Math.max(0.01, Math.min(ranges[a], ranges[b], ranges[c]));
  const maximumRange = Math.max(ranges[a], ranges[b], ranges[c]);
  const allowedEdge = Math.min(
    options.maxEdgeLengthM,
    options.baseEdgeLengthM + minimumRange * options.edgeLengthPerRange
  );
  if (Math.max(ab, bc, ca) > allowedEdge * allowedEdge) return false;

  const allowedRangeJump = Math.min(
    options.maxRangeJumpM,
    Math.max(options.baseRangeJumpM, minimumRange * options.rangeJumpRatio)
  );
  if (maximumRange - minimumRange > allowedRangeJump) return false;

  const aOffset = a * 3;
  const bOffset = b * 3;
  const cOffset = c * 3;
  const abX = positions[bOffset] - positions[aOffset];
  const abY = positions[bOffset + 1] - positions[aOffset + 1];
  const abZ = positions[bOffset + 2] - positions[aOffset + 2];
  const acX = positions[cOffset] - positions[aOffset];
  const acY = positions[cOffset + 1] - positions[aOffset + 1];
  const acZ = positions[cOffset + 2] - positions[aOffset + 2];
  const crossX = abY * acZ - abZ * acY;
  const crossY = abZ * acX - abX * acZ;
  const crossZ = abX * acY - abY * acX;
  const doubledAreaSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
  const minimumDoubledArea = options.minTriangleAreaM2 * 2;
  return doubledAreaSquared >= minimumDoubledArea * minimumDoubledArea;
}

function squaredDistance(a: number, b: number, positions: Float32Array): number {
  const aOffset = a * 3;
  const bOffset = b * 3;
  const dx = positions[aOffset] - positions[bOffset];
  const dy = positions[aOffset + 1] - positions[bOffset + 1];
  const dz = positions[aOffset + 2] - positions[bOffset + 2];
  return dx * dx + dy * dy + dz * dz;
}
