import type { ParsedCloud } from './CloudTypes';

const TWO_PI = Math.PI * 2;

export type ScanMeshBuildReason =
  | 'ok'
  | 'empty_cloud'
  | 'missing_lidar_id'
  | 'missing_ring'
  | 'missing_azimuth';

export interface StableScanMeshBuildOptions {
  maxTriangles: number;
  maxRingGap: number;
  minPointsPerRing: number;
  matchAzimuthToleranceDeg: number;
  maxAzimuthGapDeg: number;
  maxAlongEdgeM: number;
  baseAlongEdgeM: number;
  angularEdgeScale: number;
  maxCrossEdgeM: number;
  baseCrossEdgeM: number;
  crossEdgePerRange: number;
  maxDiagonalEdgeM: number;
  maxAlongRangeJumpM: number;
  baseAlongRangeJumpM: number;
  alongRangeJumpRatio: number;
  maxCrossRangeJumpM: number;
  baseCrossRangeJumpM: number;
  crossRangeJumpRatio: number;
  maxNormalAngleDeg: number;
  maxPlanarityErrorM: number;
  planarityErrorRatio: number;
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
  reason: ScanMeshBuildReason;
}

export const DEFAULT_STABLE_SCAN_MESH_OPTIONS: StableScanMeshBuildOptions = {
  maxTriangles: 400000,
  maxRingGap: 2,
  minPointsPerRing: 8,

  // TM16 points from different channels have slightly different corrected
  // azimuths. Actual source-lidar azimuth makes this tolerance deterministic;
  // unlike float timestamps it does not collapse or drift after TF fusion.
  matchAzimuthToleranceDeg: 2.0,
  // A short missing-return run may be bridged, but never a large occlusion.
  maxAzimuthGapDeg: 6.0,

  // Along-ring edges follow the angular sampling interval.
  maxAlongEdgeM: 2.5,
  baseAlongEdgeM: 0.16,
  angularEdgeScale: 2.6,

  // Floor and roof are observed at a grazing angle. Their adjacent physical
  // beams can be metres apart even though all four corners are coplanar.
  maxCrossEdgeM: 4.5,
  baseCrossEdgeM: 0.45,
  crossEdgePerRange: 0.34,
  maxDiagonalEdgeM: 5.2,

  maxAlongRangeJumpM: 2.0,
  baseAlongRangeJumpM: 0.32,
  alongRangeJumpRatio: 0.18,
  maxCrossRangeJumpM: 5.0,
  baseCrossRangeJumpM: 0.80,
  crossRangeJumpRatio: 0.42,

  // Planarity, rather than a very small edge-ratio limit, rejects false fans.
  // A grazing floor quad is naturally long and thin, so edge ratios around
  // 20-30 are valid for a 16-line lidar.
  maxNormalAngleDeg: 62.0,
  maxPlanarityErrorM: 0.26,
  planarityErrorRatio: 0.055,
  maxEdgeRatio: 45.0,
  minTriangleAreaM2: 0.00001
};

interface RingSequence {
  ring: number;
  indices: number[];
}

interface RingMatch {
  lowerPoint: number;
  upperPoint: number;
  lowerOrder: number;
  upperOrder: number;
  angle: number;
}

interface SensorRings {
  rings: Map<number, number[]>;
}

export function buildStableScanMesh(
  cloud: ParsedCloud,
  options: StableScanMeshBuildOptions = DEFAULT_STABLE_SCAN_MESH_OPTIONS
): ScanMeshBuildResult {
  if (cloud.pointCount < 4) return emptyResult('empty_cloud');
  if (!cloud.hasLidarId) return emptyResult('missing_lidar_id');
  if (!cloud.hasRing) return emptyResult('missing_ring');
  if (!cloud.hasAzimuth) return emptyResult('missing_azimuth');

  const sensors = collectSensorRings(cloud);
  const maxTriangles = Math.max(0, Math.floor(options.maxTriangles));
  const output = new Uint32Array(maxTriangles * 3);
  const ranges = buildRanges(cloud);
  const matchTolerance = degreesToRadians(options.matchAzimuthToleranceDeg);
  const maximumAzimuthGap = degreesToRadians(options.maxAzimuthGapDeg);

  let outputCount = 0;
  let rejectedTriangles = 0;
  let ringCount = 0;
  let ringPairCount = 0;

  const appendQuad = (
    lower0: number,
    upper0: number,
    lower1: number,
    upper1: number,
    ringGap: number,
    angularGap: number
  ): boolean => {
    if (outputCount + 6 > output.length) return false;
    if (
      !isQuadValid(
        lower0,
        upper0,
        lower1,
        upper1,
        ringGap,
        angularGap,
        cloud.positions,
        ranges,
        options
      )
    ) {
      rejectedTriangles += 2;
      return true;
    }

    // The split is fixed so tiny frame-to-frame range noise cannot flip the
    // diagonal and make the surface shimmer.
    output[outputCount] = lower0;
    output[outputCount + 1] = upper0;
    output[outputCount + 2] = lower1;
    output[outputCount + 3] = lower1;
    output[outputCount + 4] = upper0;
    output[outputCount + 5] = upper1;
    outputCount += 6;
    return outputCount < output.length;
  };

  outer: for (const sensor of sensors.values()) {
    const sequences = Array.from(sensor.rings.entries())
      .map(([ring, indices]) => prepareRingSequence(ring, indices, cloud))
      .filter((sequence) => sequence.indices.length >= options.minPointsPerRing)
      .sort((left, right) => left.ring - right.ring);
    ringCount += sequences.length;

    const byRing = new Map<number, RingSequence>();
    for (const sequence of sequences) byRing.set(sequence.ring, sequence);

    // Primary surface: adjacent physical beams.
    for (let sequenceIndex = 0; sequenceIndex + 1 < sequences.length; sequenceIndex += 1) {
      const lower = sequences[sequenceIndex];
      const upper = sequences[sequenceIndex + 1];
      if (upper.ring - lower.ring !== 1) continue;

      const beforePair = outputCount;
      const matches = matchRingSequences(lower, upper, cloud.azimuths, matchTolerance);
      for (let matchIndex = 1; matchIndex < matches.length; matchIndex += 1) {
        const previous = matches[matchIndex - 1];
        const current = matches[matchIndex];
        const lowerGap = positiveAngleDelta(
          cloud.azimuths[previous.lowerPoint],
          cloud.azimuths[current.lowerPoint]
        );
        const upperGap = positiveAngleDelta(
          cloud.azimuths[previous.upperPoint],
          cloud.azimuths[current.upperPoint]
        );
        const angularGap = Math.max(lowerGap, upperGap);
        if (
          !Number.isFinite(angularGap) ||
          angularGap <= 0 ||
          angularGap > maximumAzimuthGap
        ) {
          continue;
        }
        if (
          !appendQuad(
            previous.lowerPoint,
            previous.upperPoint,
            current.lowerPoint,
            current.upperPoint,
            1,
            angularGap
          )
        ) {
          break outer;
        }
      }
      if (outputCount > beforePair) ringPairCount += 1;
    }

    // Local fallback: bridge exactly one missing physical beam only where the
    // middle ring has no return near either end of the candidate quad.
    if (options.maxRingGap >= 2) {
      for (const lower of sequences) {
        const middle = byRing.get(lower.ring + 1);
        const upper = byRing.get(lower.ring + 2);
        if (!upper) continue;

        const beforePair = outputCount;
        const matches = matchRingSequences(lower, upper, cloud.azimuths, matchTolerance);
        for (let matchIndex = 1; matchIndex < matches.length; matchIndex += 1) {
          const previous = matches[matchIndex - 1];
          const current = matches[matchIndex];
          const previousAngle = previous.angle;
          const currentAngle = current.angle;
          if (
            middle &&
            hasPointNear(middle, previousAngle, cloud.azimuths, matchTolerance) &&
            hasPointNear(middle, currentAngle, cloud.azimuths, matchTolerance)
          ) {
            continue;
          }

          const lowerGap = positiveAngleDelta(
            cloud.azimuths[previous.lowerPoint],
            cloud.azimuths[current.lowerPoint]
          );
          const upperGap = positiveAngleDelta(
            cloud.azimuths[previous.upperPoint],
            cloud.azimuths[current.upperPoint]
          );
          const angularGap = Math.max(lowerGap, upperGap);
          if (
            !Number.isFinite(angularGap) ||
            angularGap <= 0 ||
            angularGap > maximumAzimuthGap * 0.75
          ) {
            continue;
          }
          if (
            !appendQuad(
              previous.lowerPoint,
              previous.upperPoint,
              current.lowerPoint,
              current.upperPoint,
              2,
              angularGap
            )
          ) {
            break outer;
          }
        }
        if (outputCount > beforePair) ringPairCount += 1;
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

function collectSensorRings(cloud: ParsedCloud): Map<number, SensorRings> {
  const sensors = new Map<number, SensorRings>();
  for (let pointIndex = 0; pointIndex < cloud.pointCount; pointIndex += 1) {
    const positionOffset = pointIndex * 3;
    const azimuth = cloud.azimuths[pointIndex];
    if (
      !Number.isFinite(cloud.positions[positionOffset]) ||
      !Number.isFinite(cloud.positions[positionOffset + 1]) ||
      !Number.isFinite(cloud.positions[positionOffset + 2]) ||
      !Number.isFinite(azimuth)
    ) {
      continue;
    }

    const lidarId = cloud.lidarIds[pointIndex];
    const ring = cloud.rings[pointIndex];
    let sensor = sensors.get(lidarId);
    if (!sensor) {
      sensor = { rings: new Map<number, number[]>() };
      sensors.set(lidarId, sensor);
    }
    let indices = sensor.rings.get(ring);
    if (!indices) {
      indices = [];
      sensor.rings.set(ring, indices);
    }
    indices.push(pointIndex);
  }
  return sensors;
}

function prepareRingSequence(
  ring: number,
  sourceIndices: number[],
  cloud: ParsedCloud
): RingSequence {
  const indices = sourceIndices
    .filter((pointIndex) => Number.isFinite(cloud.azimuths[pointIndex]))
    .sort((left, right) => {
      const delta = normalizeAngle(cloud.azimuths[left]) - normalizeAngle(cloud.azimuths[right]);
      return delta === 0 ? left - right : delta;
    });

  // Dual returns or a dense source may place several points at effectively the
  // same azimuth. Keep the nearer return so one scan column has one vertex.
  const deduplicated: number[] = [];
  const minimumSeparation = 1e-5;
  for (const pointIndex of indices) {
    const previousIndex = deduplicated[deduplicated.length - 1];
    if (previousIndex === undefined) {
      deduplicated.push(pointIndex);
      continue;
    }
    const angleDelta = Math.abs(
      normalizeAngle(cloud.azimuths[pointIndex]) -
        normalizeAngle(cloud.azimuths[previousIndex])
    );
    if (angleDelta > minimumSeparation) {
      deduplicated.push(pointIndex);
      continue;
    }

    const currentRange = usableRange(cloud, pointIndex);
    const previousRange = usableRange(cloud, previousIndex);
    if (currentRange < previousRange) {
      deduplicated[deduplicated.length - 1] = pointIndex;
    }
  }
  return { ring, indices: deduplicated };
}

function matchRingSequences(
  lower: RingSequence,
  upper: RingSequence,
  azimuths: Float32Array,
  tolerance: number
): RingMatch[] {
  if (lower.indices.length === 0 || upper.indices.length === 0) return [];

  const primaryIsLower = lower.indices.length <= upper.indices.length;
  const primary = primaryIsLower ? lower.indices : upper.indices;
  const secondary = primaryIsLower ? upper.indices : lower.indices;
  const matches: RingMatch[] = [];
  let secondaryOrder = 0;
  let lastSecondaryOrder = -1;

  for (let primaryOrder = 0; primaryOrder < primary.length; primaryOrder += 1) {
    const primaryPoint = primary[primaryOrder];
    const primaryAngle = normalizeAngle(azimuths[primaryPoint]);
    if (!Number.isFinite(primaryAngle)) continue;

    secondaryOrder = Math.max(secondaryOrder, lastSecondaryOrder + 1);
    if (secondaryOrder >= secondary.length) break;

    while (secondaryOrder + 1 < secondary.length) {
      const currentAngle = normalizeAngle(azimuths[secondary[secondaryOrder]]);
      const nextAngle = normalizeAngle(azimuths[secondary[secondaryOrder + 1]]);
      if (Math.abs(nextAngle - primaryAngle) <= Math.abs(currentAngle - primaryAngle)) {
        secondaryOrder += 1;
      } else {
        break;
      }
    }

    const secondaryPoint = secondary[secondaryOrder];
    const secondaryAngle = normalizeAngle(azimuths[secondaryPoint]);
    if (!Number.isFinite(secondaryAngle)) continue;
    if (Math.abs(secondaryAngle - primaryAngle) > tolerance) continue;

    const lowerPoint = primaryIsLower ? primaryPoint : secondaryPoint;
    const upperPoint = primaryIsLower ? secondaryPoint : primaryPoint;
    const lowerOrder = primaryIsLower ? primaryOrder : secondaryOrder;
    const upperOrder = primaryIsLower ? secondaryOrder : primaryOrder;
    matches.push({
      lowerPoint,
      upperPoint,
      lowerOrder,
      upperOrder,
      angle: (primaryAngle + secondaryAngle) * 0.5
    });
    lastSecondaryOrder = secondaryOrder;
  }

  matches.sort((left, right) => left.angle - right.angle);
  return matches;
}

function hasPointNear(
  sequence: RingSequence,
  targetAngle: number,
  azimuths: Float32Array,
  tolerance: number
): boolean {
  let low = 0;
  let high = sequence.indices.length;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    const angle = normalizeAngle(azimuths[sequence.indices[middle]]);
    if (angle < targetAngle) low = middle + 1;
    else high = middle;
  }

  for (const order of [low - 1, low, low + 1]) {
    if (order < 0 || order >= sequence.indices.length) continue;
    const angle = normalizeAngle(azimuths[sequence.indices[order]]);
    if (Math.abs(angle - targetAngle) <= tolerance) return true;
  }
  return false;
}

function buildRanges(cloud: ParsedCloud): Float32Array {
  if (cloud.hasRange) return cloud.ranges;
  const ranges = new Float32Array(cloud.pointCount);
  for (let pointIndex = 0; pointIndex < cloud.pointCount; pointIndex += 1) {
    const offset = pointIndex * 3;
    ranges[pointIndex] = Math.hypot(
      cloud.positions[offset],
      cloud.positions[offset + 1],
      cloud.positions[offset + 2]
    );
  }
  return ranges;
}

function usableRange(cloud: ParsedCloud, pointIndex: number): number {
  const supplied = cloud.hasRange ? cloud.ranges[pointIndex] : Number.NaN;
  if (Number.isFinite(supplied) && supplied > 0) return supplied;
  const offset = pointIndex * 3;
  return Math.hypot(
    cloud.positions[offset],
    cloud.positions[offset + 1],
    cloud.positions[offset + 2]
  );
}

function isQuadValid(
  lower0: number,
  upper0: number,
  lower1: number,
  upper1: number,
  ringGap: number,
  angularGap: number,
  positions: Float32Array,
  ranges: Float32Array,
  options: StableScanMeshBuildOptions
): boolean {
  const unique = new Set([lower0, upper0, lower1, upper1]);
  if (unique.size !== 4) return false;

  const alongLower = distance(lower0, lower1, positions);
  const alongUpper = distance(upper0, upper1, positions);
  const crossStart = distance(lower0, upper0, positions);
  const crossEnd = distance(lower1, upper1, positions);
  const diagonalA = distance(lower0, upper1, positions);
  const diagonalB = distance(upper0, lower1, positions);
  const allEdges = [alongLower, alongUpper, crossStart, crossEnd, diagonalA, diagonalB];
  if (allEdges.some((edge) => !Number.isFinite(edge) || edge <= 1e-6)) return false;

  const minimumRange = Math.max(
    0.01,
    Math.min(ranges[lower0], ranges[upper0], ranges[lower1], ranges[upper1])
  );
  const maximumRange = Math.max(
    ranges[lower0],
    ranges[upper0],
    ranges[lower1],
    ranges[upper1]
  );
  if (!Number.isFinite(minimumRange) || !Number.isFinite(maximumRange)) return false;

  const allowedAlongEdge = Math.min(
    options.maxAlongEdgeM,
    options.baseAlongEdgeM + maximumRange * angularGap * options.angularEdgeScale
  );
  if (Math.max(alongLower, alongUpper) > allowedAlongEdge) return false;

  const allowedCrossEdge = Math.min(
    options.maxCrossEdgeM,
    options.baseCrossEdgeM + minimumRange * options.crossEdgePerRange * ringGap
  );
  if (Math.max(crossStart, crossEnd) > allowedCrossEdge) return false;

  const allowedDiagonal = Math.min(
    options.maxDiagonalEdgeM,
    Math.hypot(allowedAlongEdge, allowedCrossEdge) * 1.35
  );
  if (Math.max(diagonalA, diagonalB) > allowedDiagonal) return false;

  const allowedAlongRangeJump = Math.min(
    options.maxAlongRangeJumpM,
    options.baseAlongRangeJumpM + minimumRange * options.alongRangeJumpRatio
  );
  if (
    Math.abs(ranges[lower1] - ranges[lower0]) > allowedAlongRangeJump ||
    Math.abs(ranges[upper1] - ranges[upper0]) > allowedAlongRangeJump
  ) {
    return false;
  }

  const allowedCrossRangeJump = Math.min(
    options.maxCrossRangeJumpM,
    options.baseCrossRangeJumpM + minimumRange * options.crossRangeJumpRatio * ringGap
  );
  if (
    Math.abs(ranges[upper0] - ranges[lower0]) > allowedCrossRangeJump ||
    Math.abs(ranges[upper1] - ranges[lower1]) > allowedCrossRangeJump
  ) {
    return false;
  }

  const first = triangleMetrics(lower0, upper0, lower1, positions);
  const second = triangleMetrics(lower1, upper0, upper1, positions);
  if (!first.valid || !second.valid) return false;
  if (first.area < options.minTriangleAreaM2 || second.area < options.minTriangleAreaM2) {
    return false;
  }
  if (first.edgeRatio > options.maxEdgeRatio || second.edgeRatio > options.maxEdgeRatio) {
    return false;
  }

  const normalDot = Math.abs(
    first.normalX * second.normalX +
      first.normalY * second.normalY +
      first.normalZ * second.normalZ
  );
  const minimumNormalDot = Math.cos(degreesToRadians(options.maxNormalAngleDeg));
  if (normalDot < minimumNormalDot) return false;

  const maximumEdge = Math.max(...allEdges);
  const allowedPlanarityError =
    options.maxPlanarityErrorM + maximumEdge * options.planarityErrorRatio;
  const fourthPointDistance = pointPlaneDistance(
    upper1,
    lower0,
    first.normalX,
    first.normalY,
    first.normalZ,
    positions
  );
  return Number.isFinite(fourthPointDistance) && fourthPointDistance <= allowedPlanarityError;
}

interface TriangleMetrics {
  valid: boolean;
  area: number;
  edgeRatio: number;
  normalX: number;
  normalY: number;
  normalZ: number;
}

function triangleMetrics(
  a: number,
  b: number,
  c: number,
  positions: Float32Array
): TriangleMetrics {
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
  const doubledArea = Math.hypot(crossX, crossY, crossZ);
  if (!Number.isFinite(doubledArea) || doubledArea <= 1e-10) {
    return {
      valid: false,
      area: 0,
      edgeRatio: Number.POSITIVE_INFINITY,
      normalX: 0,
      normalY: 0,
      normalZ: 0
    };
  }

  const ab = Math.hypot(abX, abY, abZ);
  const ac = Math.hypot(acX, acY, acZ);
  const bc = distance(b, c, positions);
  const minimumEdge = Math.max(1e-6, Math.min(ab, ac, bc));
  const maximumEdge = Math.max(ab, ac, bc);
  return {
    valid: true,
    area: doubledArea * 0.5,
    edgeRatio: maximumEdge / minimumEdge,
    normalX: crossX / doubledArea,
    normalY: crossY / doubledArea,
    normalZ: crossZ / doubledArea
  };
}

function pointPlaneDistance(
  point: number,
  planePoint: number,
  normalX: number,
  normalY: number,
  normalZ: number,
  positions: Float32Array
): number {
  const pointOffset = point * 3;
  const planeOffset = planePoint * 3;
  const dx = positions[pointOffset] - positions[planeOffset];
  const dy = positions[pointOffset + 1] - positions[planeOffset + 1];
  const dz = positions[pointOffset + 2] - positions[planeOffset + 2];
  return Math.abs(dx * normalX + dy * normalY + dz * normalZ);
}

function distance(a: number, b: number, positions: Float32Array): number {
  const aOffset = a * 3;
  const bOffset = b * 3;
  return Math.hypot(
    positions[aOffset] - positions[bOffset],
    positions[aOffset + 1] - positions[bOffset + 1],
    positions[aOffset + 2] - positions[bOffset + 2]
  );
}

function normalizeAngle(angle: number): number {
  if (!Number.isFinite(angle)) return Number.NaN;
  const normalized = angle % TWO_PI;
  return normalized < 0 ? normalized + TWO_PI : normalized;
}

function positiveAngleDelta(from: number, to: number): number {
  const normalizedFrom = normalizeAngle(from);
  const normalizedTo = normalizeAngle(to);
  if (!Number.isFinite(normalizedFrom) || !Number.isFinite(normalizedTo)) {
    return Number.NaN;
  }
  const delta = normalizedTo - normalizedFrom;
  return delta >= 0 ? delta : delta + TWO_PI;
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

function emptyResult(reason: Exclude<ScanMeshBuildReason, 'ok'>): ScanMeshBuildResult {
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
