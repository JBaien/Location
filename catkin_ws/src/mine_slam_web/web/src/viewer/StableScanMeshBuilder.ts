import type { ParsedCloud } from './CloudTypes';

export interface StableScanMeshBuildOptions {
  minColumnsPerScan: number;
  maxColumnsPerScan: number;
  targetCellFillRatio: number;
  maxColumnBridge: number;
  maxTriangles: number;
  maxRingGap: number;
  minPointsPerRing: number;
  minFiniteTimeRatio: number;
  minTimeSpan: number;
  maxEdgeLengthM: number;
  baseEdgeLengthM: number;
  edgeLengthPerRange: number;
  maxRangeJumpM: number;
  baseRangeJumpM: number;
  rangeJumpRatio: number;
  maxEdgeRatio: number;
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

export const DEFAULT_STABLE_SCAN_MESH_OPTIONS: StableScanMeshBuildOptions = {
  // Spatial voxel filtering removes many more samples from close floor/roof
  // returns than from distant walls. Choose the angular grid from the lower
  // quartile ring density instead of forcing every frame into 720 columns.
  minColumnsPerScan: 180,
  maxColumnsPerScan: 480,
  targetCellFillRatio: 0.74,
  // Bridge only short angular gaps. At 360 columns this is at most 4 degrees.
  maxColumnBridge: 4,
  maxTriangles: 300000,
  // A missing beam may be bridged by the next physical ring when geometry is
  // continuous. Normal adjacent-ring quads remain preferred.
  maxRingGap: 2,
  minPointsPerRing: 24,
  minFiniteTimeRatio: 0.65,
  minTimeSpan: 1e-5,

  maxEdgeLengthM: 1.2,
  baseEdgeLengthM: 0.22,
  edgeLengthPerRange: 0.10,
  maxRangeJumpM: 1.0,
  baseRangeJumpM: 0.26,
  rangeJumpRatio: 0.15,
  maxEdgeRatio: 6.0,
  minTriangleAreaM2: 0.00002
};

interface SensorBuckets {
  pointIndices: number[];
  rings: Map<number, number[]>;
}

interface TimeDomain {
  usable: boolean;
  minimum: number;
  maximum: number;
}

interface RingGrid {
  ring: number;
  cells: Int32Array;
  validCellCount: number;
}

export function buildStableScanMesh(
  cloud: ParsedCloud,
  options: StableScanMeshBuildOptions = DEFAULT_STABLE_SCAN_MESH_OPTIONS
): ScanMeshBuildResult {
  if (cloud.pointCount < 4) return emptyResult('empty_cloud');
  if (!cloud.hasLidarId) return emptyResult('missing_lidar_id');
  if (!cloud.hasRing) return emptyResult('missing_ring');

  const sensorBuckets = collectSensorBuckets(cloud);
  const maxTriangles = Math.max(0, Math.floor(options.maxTriangles));
  const output = new Uint32Array(maxTriangles * 3);
  const ranges = computeRanges(cloud.positions, cloud.pointCount);
  let outputCount = 0;
  let rejectedTriangles = 0;
  let ringCount = 0;
  let ringPairCount = 0;

  const appendQuad = (a0: number, b0: number, a1: number, b1: number): boolean => {
    if (outputCount + 6 > output.length) return false;
    const firstValid = isTriangleValid(a0, b0, a1, cloud.positions, ranges, options);
    const secondValid = isTriangleValid(a1, b0, b1, cloud.positions, ranges, options);
    // Quads are atomic. Keeping only one half was the source of the isolated
    // fins/hourglass shapes seen in the real tunnel recording.
    if (!firstValid || !secondValid) {
      rejectedTriangles += 2;
      return true;
    }
    output[outputCount] = a0;
    output[outputCount + 1] = b0;
    output[outputCount + 2] = a1;
    output[outputCount + 3] = a1;
    output[outputCount + 4] = b0;
    output[outputCount + 5] = b1;
    outputCount += 6;
    return outputCount < output.length;
  };

  outer: for (const buckets of sensorBuckets.values()) {
    const timeDomain = estimateTimeDomain(buckets.pointIndices, cloud, options);
    const columnCount = chooseColumnCount(buckets.rings, options);
    const ringGrids = Array.from(buckets.rings.entries())
      .map(([ring, indices]) =>
        buildRingGrid(ring, indices, cloud, ranges, timeDomain, columnCount, options)
      )
      .filter((grid) => grid.validCellCount >= 2)
      .sort((left, right) => left.ring - right.ring);

    ringCount += ringGrids.length;

    // First build the normal adjacent-ring surface.
    for (let lowerIndex = 0; lowerIndex + 1 < ringGrids.length; lowerIndex += 1) {
      const lower = ringGrids[lowerIndex];
      const upper = ringGrids[lowerIndex + 1];
      if (upper.ring - lower.ring !== 1) continue;
      const beforePair = outputCount;
      if (!connectRingPair(lower, upper, null, options.maxColumnBridge, appendQuad)) {
        break outer;
      }
      if (outputCount > beforePair) ringPairCount += 1;
    }

    // Then fill a missing physical beam only where the intermediate ring does
    // not already provide a complete local quad. Geometry checks still gate
    // every fallback quad.
    if (options.maxRingGap >= 2) {
      for (let lowerIndex = 0; lowerIndex + 2 < ringGrids.length; lowerIndex += 1) {
        const lower = ringGrids[lowerIndex];
        const middle = ringGrids[lowerIndex + 1];
        const upper = ringGrids[lowerIndex + 2];
        if (middle.ring - lower.ring !== 1 || upper.ring - middle.ring !== 1) {
          continue;
        }
        const beforePair = outputCount;
        if (!connectRingPair(lower, upper, middle, Math.min(2, options.maxColumnBridge), appendQuad)) {
          break outer;
        }
        if (outputCount > beforePair) ringPairCount += 1;
      }
    }
  }

  return {
    indices: output.slice(0, outputCount),
    triangleCount: outputCount / 3,
    sensorCount: sensorBuckets.size,
    ringCount,
    ringPairCount,
    rejectedTriangles,
    reason: 'ok'
  };
}

function connectRingPair(
  lower: RingGrid,
  upper: RingGrid,
  blockingMiddle: RingGrid | null,
  maxColumnBridge: number,
  appendQuad: (a0: number, b0: number, a1: number, b1: number) => boolean
): boolean {
  const columns = Math.min(lower.cells.length, upper.cells.length);
  let previousColumn = -1;
  for (let column = 0; column < columns; column += 1) {
    if (lower.cells[column] < 0 || upper.cells[column] < 0) continue;
    if (previousColumn >= 0) {
      const gap = column - previousColumn;
      if (gap <= maxColumnBridge) {
        const middleBlocksFallback =
          blockingMiddle !== null &&
          blockingMiddle.cells[previousColumn] >= 0 &&
          blockingMiddle.cells[column] >= 0;
        if (!middleBlocksFallback) {
          if (
            !appendQuad(
              lower.cells[previousColumn],
              upper.cells[previousColumn],
              lower.cells[column],
              upper.cells[column]
            )
          ) {
            return false;
          }
        }
      }
    }
    previousColumn = column;
  }
  return true;
}

function chooseColumnCount(
  rings: Map<number, number[]>,
  options: StableScanMeshBuildOptions
): number {
  const counts = Array.from(rings.values())
    .map((indices) => indices.length)
    .filter((count) => count >= options.minPointsPerRing)
    .sort((left, right) => left - right);
  if (counts.length === 0) return options.minColumnsPerScan;

  const lowerQuartile = counts[Math.floor((counts.length - 1) * 0.25)];
  const targetFill = Math.max(0.25, Math.min(0.95, options.targetCellFillRatio));
  const estimated = Math.round(lowerQuartile / targetFill);
  const clamped = Math.max(
    options.minColumnsPerScan,
    Math.min(options.maxColumnsPerScan, estimated)
  );
  // A multiple of eight keeps the angular bins stable and inexpensive.
  return Math.max(16, Math.round(clamped / 8) * 8);
}

function collectSensorBuckets(cloud: ParsedCloud): Map<number, SensorBuckets> {
  const sensors = new Map<number, SensorBuckets>();
  const positions = cloud.positions;
  for (let pointIndex = 0; pointIndex < cloud.pointCount; pointIndex += 1) {
    const offset = pointIndex * 3;
    if (
      !Number.isFinite(positions[offset]) ||
      !Number.isFinite(positions[offset + 1]) ||
      !Number.isFinite(positions[offset + 2])
    ) {
      continue;
    }

    const lidarId = cloud.lidarIds[pointIndex];
    const ring = cloud.rings[pointIndex];
    let buckets = sensors.get(lidarId);
    if (!buckets) {
      buckets = { pointIndices: [], rings: new Map<number, number[]>() };
      sensors.set(lidarId, buckets);
    }
    buckets.pointIndices.push(pointIndex);
    let ringIndices = buckets.rings.get(ring);
    if (!ringIndices) {
      ringIndices = [];
      buckets.rings.set(ring, ringIndices);
    }
    ringIndices.push(pointIndex);
  }
  return sensors;
}

function estimateTimeDomain(
  indices: number[],
  cloud: ParsedCloud,
  options: StableScanMeshBuildOptions
): TimeDomain {
  if (!cloud.hasTime || indices.length < 4) {
    return { usable: false, minimum: 0, maximum: 0 };
  }

  let finiteCount = 0;
  let minimum = Number.POSITIVE_INFINITY;
  let maximum = Number.NEGATIVE_INFINITY;
  let previous = Number.NEGATIVE_INFINITY;
  let inversions = 0;
  for (const pointIndex of indices) {
    const value = cloud.times[pointIndex];
    if (!Number.isFinite(value)) continue;
    finiteCount += 1;
    minimum = Math.min(minimum, value);
    maximum = Math.max(maximum, value);
    if (value + Number.EPSILON < previous) inversions += 1;
    previous = value;
  }

  const enoughFinite =
    finiteCount >= Math.max(4, Math.floor(indices.length * options.minFiniteTimeRatio));
  const mostlyMonotonic = inversions <= Math.max(3, Math.floor(finiteCount * 0.02));
  const usable =
    enoughFinite && mostlyMonotonic && maximum - minimum >= options.minTimeSpan;
  return { usable, minimum, maximum };
}

function buildRingGrid(
  ring: number,
  sourceIndices: number[],
  cloud: ParsedCloud,
  ranges: Float32Array,
  timeDomain: TimeDomain,
  columnCount: number,
  options: StableScanMeshBuildOptions
): RingGrid {
  const columns = Math.max(16, Math.floor(columnCount));
  const cells = new Int32Array(columns);
  cells.fill(-1);
  const scores = new Float64Array(columns);
  scores.fill(Number.POSITIVE_INFINITY);

  if (sourceIndices.length < options.minPointsPerRing) {
    return { ring, cells, validCellCount: 0 };
  }

  if (timeDomain.usable) {
    const span = timeDomain.maximum - timeDomain.minimum;
    for (const pointIndex of sourceIndices) {
      const pointTime = cloud.times[pointIndex];
      if (!Number.isFinite(pointTime)) continue;
      const normalized = (pointTime - timeDomain.minimum) / span;
      if (normalized < -0.001 || normalized > 1.001) continue;
      placePointInCell(pointIndex, normalized, cells, scores, ranges);
    }
  } else {
    const denominator = Math.max(1, sourceIndices.length - 1);
    for (let order = 0; order < sourceIndices.length; order += 1) {
      placePointInCell(sourceIndices[order], order / denominator, cells, scores, ranges);
    }
  }

  let validCellCount = 0;
  for (let column = 0; column < cells.length; column += 1) {
    if (cells[column] >= 0) validCellCount += 1;
  }
  return { ring, cells, validCellCount };
}

function placePointInCell(
  pointIndex: number,
  normalizedPosition: number,
  cells: Int32Array,
  scores: Float64Array,
  ranges: Float32Array
): void {
  const bounded = Math.max(0, Math.min(1, normalizedPosition));
  const scaled = bounded * (cells.length - 1);
  const column = Math.round(scaled);
  const phaseError = Math.abs(scaled - column);
  const previous = cells[column];
  const betterPhase = phaseError + 1e-9 < scores[column];
  const equalPhaseAndNearer =
    Math.abs(phaseError - scores[column]) <= 1e-9 &&
    (previous < 0 || ranges[pointIndex] < ranges[previous]);
  if (previous < 0 || betterPhase || equalPhaseAndNearer) {
    cells[column] = pointIndex;
    scores[column] = phaseError;
  }
}

function computeRanges(positions: Float32Array, pointCount: number): Float32Array {
  const ranges = new Float32Array(pointCount);
  for (let pointIndex = 0; pointIndex < pointCount; pointIndex += 1) {
    const offset = pointIndex * 3;
    ranges[pointIndex] = Math.hypot(
      positions[offset],
      positions[offset + 1],
      positions[offset + 2]
    );
  }
  return ranges;
}

function isTriangleValid(
  a: number,
  b: number,
  c: number,
  positions: Float32Array,
  ranges: Float32Array,
  options: StableScanMeshBuildOptions
): boolean {
  if (a === b || b === c || a === c) return false;

  const ab = squaredDistance(a, b, positions);
  const bc = squaredDistance(b, c, positions);
  const ca = squaredDistance(c, a, positions);
  if (!Number.isFinite(ab) || !Number.isFinite(bc) || !Number.isFinite(ca)) {
    return false;
  }

  const minimumEdgeSquared = Math.min(ab, bc, ca);
  const maximumEdgeSquared = Math.max(ab, bc, ca);
  if (minimumEdgeSquared <= 1e-8) return false;

  const minimumRange = Math.max(0.01, Math.min(ranges[a], ranges[b], ranges[c]));
  const maximumRange = Math.max(ranges[a], ranges[b], ranges[c]);
  const allowedEdge = Math.min(
    options.maxEdgeLengthM,
    options.baseEdgeLengthM + minimumRange * options.edgeLengthPerRange
  );
  if (maximumEdgeSquared > allowedEdge * allowedEdge) return false;

  const allowedRangeJump = Math.min(
    options.maxRangeJumpM,
    Math.max(options.baseRangeJumpM, minimumRange * options.rangeJumpRatio)
  );
  if (maximumRange - minimumRange > allowedRangeJump) return false;

  const edgeRatioSquared = maximumEdgeSquared / minimumEdgeSquared;
  if (edgeRatioSquared > options.maxEdgeRatio * options.maxEdgeRatio) return false;

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
