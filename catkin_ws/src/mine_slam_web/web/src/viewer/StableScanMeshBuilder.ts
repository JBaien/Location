import type { ParsedCloud } from './CloudTypes';

export interface StableScanMeshBuildOptions {
  columnsPerScan: number;
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
  allowPartialCells: boolean;
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
  // A fixed angular grid makes the topology deterministic between frames.
  // The previous nearest-time pairing could select a different diagonal or
  // neighbour whenever a few returns disappeared, which caused visible jumps.
  columnsPerScan: 720,
  maxTriangles: 300000,
  maxRingGap: 1,
  minPointsPerRing: 24,
  minFiniteTimeRatio: 0.65,
  minTimeSpan: 1e-5,

  // Floor and roof returns are often observed at a grazing angle, so their
  // adjacent samples are farther apart than wall samples. The fixed scan grid
  // lets these limits be relaxed without connecting across missing azimuths.
  maxEdgeLengthM: 1.6,
  baseEdgeLengthM: 0.24,
  edgeLengthPerRange: 0.12,
  maxRangeJumpM: 1.4,
  baseRangeJumpM: 0.30,
  rangeJumpRatio: 0.18,

  // Reject the long, needle-like fans that appeared near occlusion edges.
  maxEdgeRatio: 8.0,
  minTriangleAreaM2: 0.00002,
  allowPartialCells: true
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

  const appendTriangle = (a: number, b: number, c: number): boolean => {
    if (outputCount + 3 > output.length) return false;
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

  outer: for (const buckets of sensorBuckets.values()) {
    const timeDomain = estimateTimeDomain(buckets.pointIndices, cloud, options);
    const ringGrids = Array.from(buckets.rings.entries())
      .map(([ring, indices]) =>
        buildRingGrid(ring, indices, cloud, ranges, timeDomain, options)
      )
      .filter((grid) => grid.validCellCount >= 2)
      .sort((left, right) => left.ring - right.ring);

    ringCount += ringGrids.length;
    for (let pairIndex = 0; pairIndex + 1 < ringGrids.length; pairIndex += 1) {
      const lower = ringGrids[pairIndex];
      const upper = ringGrids[pairIndex + 1];
      if (upper.ring - lower.ring > options.maxRingGap) continue;

      const beforePair = outputCount;
      const columns = Math.min(lower.cells.length, upper.cells.length);
      for (let column = 0; column + 1 < columns; column += 1) {
        const a0 = lower.cells[column];
        const a1 = lower.cells[column + 1];
        const b0 = upper.cells[column];
        const b1 = upper.cells[column + 1];

        const validCount =
          Number(a0 >= 0) + Number(a1 >= 0) + Number(b0 >= 0) + Number(b1 >= 0);

        if (validCount === 4) {
          // Keep the diagonal deterministic. Choosing the shorter diagonal on
          // every frame makes topology flip when noisy points move slightly.
          if (!appendTriangle(a0, b0, a1)) break outer;
          if (!appendTriangle(a1, b0, b1)) break outer;
          continue;
        }

        if (!options.allowPartialCells || validCount !== 3) continue;
        if (a0 < 0) {
          if (!appendTriangle(a1, b0, b1)) break outer;
        } else if (a1 < 0) {
          if (!appendTriangle(a0, b0, b1)) break outer;
        } else if (b0 < 0) {
          if (!appendTriangle(a0, b1, a1)) break outer;
        } else {
          if (!appendTriangle(a0, b0, a1)) break outer;
        }
      }
      if (outputCount > beforePair) ringPairCount += 1;
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
  options: StableScanMeshBuildOptions
): RingGrid {
  const columns = Math.max(16, Math.floor(options.columnsPerScan));
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
