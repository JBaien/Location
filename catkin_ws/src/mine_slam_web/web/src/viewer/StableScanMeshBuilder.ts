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
  minPointsPerRing: number;

  minMatchToleranceDeg: number;
  maxMatchToleranceDeg: number;
  matchToleranceFactor: number;
  minAzimuthGapDeg: number;
  maxAzimuthGapDeg: number;
  azimuthGapFactor: number;

  maxAlongEdgeM: number;
  baseAlongEdgeM: number;
  angularEdgeScale: number;
  maxCrossEdgeM: number;
  maxDiagonalEdgeM: number;

  maxAlongRangeJumpM: number;
  baseAlongRangeJumpM: number;
  alongRangeJumpRatio: number;

  maxTriangleNormalAngleDeg: number;
  maxRunNormalAngleDeg: number;
  maxOppositeEdgeAngleDeg: number;
  maxOppositeEdgeLengthRatio: number;
  maxPlanarityErrorM: number;
  planarityErrorRatio: number;
  maxEdgeRatio: number;
  minTriangleAreaM2: number;
  minRunQuads: number;
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
  minPointsPerRing: 8,

  // Match only genuinely corresponding source-lidar azimuths. The earlier
  // fixed 2 degree tolerance could pair points several native columns apart.
  minMatchToleranceDeg: 0.12,
  maxMatchToleranceDeg: 0.65,
  matchToleranceFactor: 1.8,

  // Native TM16 spacing is normally about 0.15-0.25 degrees at 10 Hz. Permit
  // a few missing returns, but never bridge the multi-degree gaps that created
  // the large floating polygons visible in the real bag.
  minAzimuthGapDeg: 0.35,
  maxAzimuthGapDeg: 1.20,
  azimuthGapFactor: 4.5,

  // A floor or roof quad can be long across adjacent vertical beams because a
  // 16-line lidar observes those surfaces at a grazing angle. Large cross-ring
  // edges are therefore accepted only after the planarity and strip-support
  // checks below pass.
  maxAlongEdgeM: 2.4,
  baseAlongEdgeM: 0.12,
  angularEdgeScale: 3.2,
  maxCrossEdgeM: 8.0,
  maxDiagonalEdgeM: 9.0,

  maxAlongRangeJumpM: 2.5,
  baseAlongRangeJumpM: 0.30,
  alongRangeJumpRatio: 0.16,

  maxTriangleNormalAngleDeg: 32.0,
  maxRunNormalAngleDeg: 28.0,
  maxOppositeEdgeAngleDeg: 42.0,
  maxOppositeEdgeLengthRatio: 4.0,
  maxPlanarityErrorM: 0.10,
  planarityErrorRatio: 0.035,
  maxEdgeRatio: 90.0,
  minTriangleAreaM2: 0.00001,

  // Never publish an isolated quad. A real wall/floor/roof patch produces a
  // run of neighboring quads; isolated candidates are usually occlusion fans.
  minRunQuads: 2
};

interface RingSequence {
  ring: number;
  indices: number[];
  medianStep: number;
}

interface RingMatch {
  lowerPoint: number;
  upperPoint: number;
  angle: number;
}

interface SensorRings {
  rings: Map<number, number[]>;
}

interface QuadCandidate {
  lower0: number;
  upper0: number;
  lower1: number;
  upper1: number;
  normalX: number;
  normalY: number;
  normalZ: number;
}

interface QuadEvaluation {
  valid: boolean;
  normalX: number;
  normalY: number;
  normalZ: number;
}

interface TriangleMetrics {
  valid: boolean;
  area: number;
  edgeRatio: number;
  normalX: number;
  normalY: number;
  normalZ: number;
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

  let outputCount = 0;
  let rejectedTriangles = 0;
  let ringCount = 0;
  let ringPairCount = 0;

  const appendQuad = (candidate: QuadCandidate): boolean => {
    if (outputCount + 6 > output.length) return false;
    output[outputCount] = candidate.lower0;
    output[outputCount + 1] = candidate.upper0;
    output[outputCount + 2] = candidate.lower1;
    output[outputCount + 3] = candidate.lower1;
    output[outputCount + 4] = candidate.upper0;
    output[outputCount + 5] = candidate.upper1;
    outputCount += 6;
    return outputCount < output.length;
  };

  outer: for (const sensor of sensors.values()) {
    const sequences = Array.from(sensor.rings.entries())
      .map(([ring, indices]) => prepareRingSequence(ring, indices, cloud))
      .filter((sequence) => sequence.indices.length >= options.minPointsPerRing)
      .sort((left, right) => left.ring - right.ring);
    ringCount += sequences.length;

    for (let sequenceIndex = 0; sequenceIndex + 1 < sequences.length; sequenceIndex += 1) {
      const lower = sequences[sequenceIndex];
      const upper = sequences[sequenceIndex + 1];

      // Only neighboring physical beams are connected. The former ring+2
      // fallback created very large polygons across locally missing returns.
      if (upper.ring - lower.ring !== 1) continue;

      const representativeStep = robustMaximumStep(lower.medianStep, upper.medianStep);
      const matchTolerance = clamp(
        representativeStep * options.matchToleranceFactor,
        degreesToRadians(options.minMatchToleranceDeg),
        degreesToRadians(options.maxMatchToleranceDeg)
      );
      const maximumAzimuthGap = clamp(
        representativeStep * options.azimuthGapFactor,
        degreesToRadians(options.minAzimuthGapDeg),
        degreesToRadians(options.maxAzimuthGapDeg)
      );
      const matches = matchRingSequences(lower, upper, cloud.azimuths, matchTolerance);
      if (matches.length < options.minRunQuads + 1) continue;

      let run: QuadCandidate[] = [];
      let pairAccepted = false;

      const flushRun = (): boolean => {
        if (run.length < options.minRunQuads) {
          rejectedTriangles += run.length * 2;
          run = [];
          return true;
        }
        for (const candidate of run) {
          if (!appendQuad(candidate)) return false;
        }
        pairAccepted = true;
        run = [];
        return true;
      };

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
          if (!flushRun()) break outer;
          continue;
        }

        const evaluation = evaluateQuad(
          previous.lowerPoint,
          previous.upperPoint,
          current.lowerPoint,
          current.upperPoint,
          angularGap,
          cloud.positions,
          ranges,
          options
        );
        if (!evaluation.valid) {
          rejectedTriangles += 2;
          if (!flushRun()) break outer;
          continue;
        }

        const candidate: QuadCandidate = {
          lower0: previous.lowerPoint,
          upper0: previous.upperPoint,
          lower1: current.lowerPoint,
          upper1: current.upperPoint,
          normalX: evaluation.normalX,
          normalY: evaluation.normalY,
          normalZ: evaluation.normalZ
        };

        const previousCandidate = run[run.length - 1];
        if (
          previousCandidate &&
          !normalsAgree(
            previousCandidate.normalX,
            previousCandidate.normalY,
            previousCandidate.normalZ,
            candidate.normalX,
            candidate.normalY,
            candidate.normalZ,
            options.maxRunNormalAngleDeg
          )
        ) {
          if (!flushRun()) break outer;
        }
        run.push(candidate);
      }

      if (!flushRun()) break outer;
      if (pairAccepted) ringPairCount += 1;
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
  const sorted = sourceIndices
    .filter((pointIndex) => Number.isFinite(cloud.azimuths[pointIndex]))
    .sort((left, right) => {
      const delta =
        normalizeAngle(cloud.azimuths[left]) - normalizeAngle(cloud.azimuths[right]);
      return delta === 0 ? left - right : delta;
    });

  // One range-image column may contain duplicate or dual returns. Keep the
  // nearer return, otherwise duplicate angles create zero-width quads.
  const indices: number[] = [];
  const minimumSeparation = 1e-5;
  for (const pointIndex of sorted) {
    const previousIndex = indices[indices.length - 1];
    if (previousIndex === undefined) {
      indices.push(pointIndex);
      continue;
    }
    const angleDelta = Math.abs(
      normalizeAngle(cloud.azimuths[pointIndex]) -
        normalizeAngle(cloud.azimuths[previousIndex])
    );
    if (angleDelta > minimumSeparation) {
      indices.push(pointIndex);
      continue;
    }
    if (usableRange(cloud, pointIndex) < usableRange(cloud, previousIndex)) {
      indices[indices.length - 1] = pointIndex;
    }
  }

  return {
    ring,
    indices,
    medianStep: estimateMedianStep(indices, cloud.azimuths)
  };
}

function estimateMedianStep(indices: number[], azimuths: Float32Array): number {
  const deltas: number[] = [];
  const maximumSampleGap = degreesToRadians(2.0);
  for (let order = 1; order < indices.length; order += 1) {
    const delta = positiveAngleDelta(
      azimuths[indices[order - 1]],
      azimuths[indices[order]]
    );
    if (Number.isFinite(delta) && delta > 1e-6 && delta <= maximumSampleGap) {
      deltas.push(delta);
    }
  }
  if (deltas.length === 0) return degreesToRadians(0.20);
  deltas.sort((left, right) => left - right);
  return deltas[Math.floor(deltas.length / 2)];
}

function matchRingSequences(
  lower: RingSequence,
  upper: RingSequence,
  azimuths: Float32Array,
  tolerance: number
): RingMatch[] {
  const matches: RingMatch[] = [];
  let lowerOrder = 0;
  let upperOrder = 0;

  while (lowerOrder < lower.indices.length && upperOrder < upper.indices.length) {
    const lowerPoint = lower.indices[lowerOrder];
    const upperPoint = upper.indices[upperOrder];
    const lowerAngle = normalizeAngle(azimuths[lowerPoint]);
    const upperAngle = normalizeAngle(azimuths[upperPoint]);
    if (!Number.isFinite(lowerAngle)) {
      lowerOrder += 1;
      continue;
    }
    if (!Number.isFinite(upperAngle)) {
      upperOrder += 1;
      continue;
    }

    const difference = lowerAngle - upperAngle;
    if (Math.abs(difference) <= tolerance) {
      // Compare one-step alternatives before committing the one-to-one match.
      const nextLowerDifference =
        lowerOrder + 1 < lower.indices.length
          ? Math.abs(
              normalizeAngle(azimuths[lower.indices[lowerOrder + 1]]) - upperAngle
            )
          : Number.POSITIVE_INFINITY;
      const nextUpperDifference =
        upperOrder + 1 < upper.indices.length
          ? Math.abs(
              lowerAngle -
                normalizeAngle(azimuths[upper.indices[upperOrder + 1]])
            )
          : Number.POSITIVE_INFINITY;

      if (nextLowerDifference + 1e-9 < Math.abs(difference)) {
        lowerOrder += 1;
        continue;
      }
      if (nextUpperDifference + 1e-9 < Math.abs(difference)) {
        upperOrder += 1;
        continue;
      }

      matches.push({
        lowerPoint,
        upperPoint,
        angle: (lowerAngle + upperAngle) * 0.5
      });
      lowerOrder += 1;
      upperOrder += 1;
    } else if (difference < 0) {
      lowerOrder += 1;
    } else {
      upperOrder += 1;
    }
  }

  return matches;
}

function evaluateQuad(
  lower0: number,
  upper0: number,
  lower1: number,
  upper1: number,
  angularGap: number,
  positions: Float32Array,
  ranges: Float32Array,
  options: StableScanMeshBuildOptions
): QuadEvaluation {
  if (new Set([lower0, upper0, lower1, upper1]).size !== 4) {
    return invalidQuad();
  }

  const alongLower = vectorBetween(lower0, lower1, positions);
  const alongUpper = vectorBetween(upper0, upper1, positions);
  const crossStart = vectorBetween(lower0, upper0, positions);
  const crossEnd = vectorBetween(lower1, upper1, positions);
  const diagonalA = vectorBetween(lower0, upper1, positions);
  const diagonalB = vectorBetween(upper0, lower1, positions);
  const vectors = [alongLower, alongUpper, crossStart, crossEnd, diagonalA, diagonalB];
  if (vectors.some((vector) => !vector.valid || vector.length <= 1e-6)) {
    return invalidQuad();
  }

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
  if (!Number.isFinite(minimumRange) || !Number.isFinite(maximumRange)) {
    return invalidQuad();
  }

  const allowedAlongEdge = Math.min(
    options.maxAlongEdgeM,
    options.baseAlongEdgeM + maximumRange * angularGap * options.angularEdgeScale
  );
  if (Math.max(alongLower.length, alongUpper.length) > allowedAlongEdge) {
    return invalidQuad();
  }
  if (Math.max(crossStart.length, crossEnd.length) > options.maxCrossEdgeM) {
    return invalidQuad();
  }
  if (Math.max(diagonalA.length, diagonalB.length) > options.maxDiagonalEdgeM) {
    return invalidQuad();
  }

  const allowedAlongRangeJump = Math.min(
    options.maxAlongRangeJumpM,
    options.baseAlongRangeJumpM + minimumRange * options.alongRangeJumpRatio
  );
  if (
    Math.abs(ranges[lower1] - ranges[lower0]) > allowedAlongRangeJump ||
    Math.abs(ranges[upper1] - ranges[upper0]) > allowedAlongRangeJump
  ) {
    return invalidQuad();
  }

  if (
    !vectorsAgree(alongLower, alongUpper, options.maxOppositeEdgeAngleDeg) ||
    !vectorsAgree(crossStart, crossEnd, options.maxOppositeEdgeAngleDeg)
  ) {
    return invalidQuad();
  }
  if (
    edgeLengthRatio(alongLower.length, alongUpper.length) >
      options.maxOppositeEdgeLengthRatio ||
    edgeLengthRatio(crossStart.length, crossEnd.length) >
      options.maxOppositeEdgeLengthRatio
  ) {
    return invalidQuad();
  }

  const first = triangleMetrics(lower0, upper0, lower1, positions);
  const second = triangleMetrics(lower1, upper0, upper1, positions);
  if (!first.valid || !second.valid) return invalidQuad();
  if (first.area < options.minTriangleAreaM2 || second.area < options.minTriangleAreaM2) {
    return invalidQuad();
  }
  if (first.edgeRatio > options.maxEdgeRatio || second.edgeRatio > options.maxEdgeRatio) {
    return invalidQuad();
  }
  if (
    !normalsAgree(
      first.normalX,
      first.normalY,
      first.normalZ,
      second.normalX,
      second.normalY,
      second.normalZ,
      options.maxTriangleNormalAngleDeg
    )
  ) {
    return invalidQuad();
  }

  const maximumEdge = Math.max(...vectors.map((vector) => vector.length));
  const allowedPlanarityError =
    options.maxPlanarityErrorM + maximumEdge * options.planarityErrorRatio;
  const firstPlaneError = pointPlaneDistance(
    upper1,
    lower0,
    first.normalX,
    first.normalY,
    first.normalZ,
    positions
  );
  const secondPlaneError = pointPlaneDistance(
    lower0,
    lower1,
    second.normalX,
    second.normalY,
    second.normalZ,
    positions
  );
  if (
    !Number.isFinite(firstPlaneError) ||
    !Number.isFinite(secondPlaneError) ||
    Math.max(firstPlaneError, secondPlaneError) > allowedPlanarityError
  ) {
    return invalidQuad();
  }

  let secondNormalX = second.normalX;
  let secondNormalY = second.normalY;
  let secondNormalZ = second.normalZ;
  const normalDot =
    first.normalX * secondNormalX +
    first.normalY * secondNormalY +
    first.normalZ * secondNormalZ;
  if (normalDot < 0) {
    secondNormalX = -secondNormalX;
    secondNormalY = -secondNormalY;
    secondNormalZ = -secondNormalZ;
  }

  const normalX = first.normalX + secondNormalX;
  const normalY = first.normalY + secondNormalY;
  const normalZ = first.normalZ + secondNormalZ;
  const normalLength = Math.hypot(normalX, normalY, normalZ);
  if (!Number.isFinite(normalLength) || normalLength <= 1e-9) return invalidQuad();

  return {
    valid: true,
    normalX: normalX / normalLength,
    normalY: normalY / normalLength,
    normalZ: normalZ / normalLength
  };
}

interface VectorMetrics {
  valid: boolean;
  x: number;
  y: number;
  z: number;
  length: number;
}

function vectorBetween(
  from: number,
  to: number,
  positions: Float32Array
): VectorMetrics {
  const fromOffset = from * 3;
  const toOffset = to * 3;
  const x = positions[toOffset] - positions[fromOffset];
  const y = positions[toOffset + 1] - positions[fromOffset + 1];
  const z = positions[toOffset + 2] - positions[fromOffset + 2];
  const length = Math.hypot(x, y, z);
  return {
    valid: Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z) && Number.isFinite(length),
    x,
    y,
    z,
    length
  };
}

function vectorsAgree(
  first: VectorMetrics,
  second: VectorMetrics,
  maximumAngleDeg: number
): boolean {
  if (first.length <= 1e-9 || second.length <= 1e-9) return false;
  const dot =
    (first.x * second.x + first.y * second.y + first.z * second.z) /
    (first.length * second.length);
  return Number.isFinite(dot) && dot >= Math.cos(degreesToRadians(maximumAngleDeg));
}

function edgeLengthRatio(first: number, second: number): number {
  const minimum = Math.max(1e-6, Math.min(first, second));
  return Math.max(first, second) / minimum;
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
  const bc = vectorBetween(b, c, positions).length;
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

function normalsAgree(
  firstX: number,
  firstY: number,
  firstZ: number,
  secondX: number,
  secondY: number,
  secondZ: number,
  maximumAngleDeg: number
): boolean {
  const dot = Math.abs(firstX * secondX + firstY * secondY + firstZ * secondZ);
  return Number.isFinite(dot) && dot >= Math.cos(degreesToRadians(maximumAngleDeg));
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

function buildRanges(cloud: ParsedCloud): Float32Array {
  if (cloud.hasRange) return cloud.ranges;
  const ranges = new Float32Array(cloud.pointCount);
  for (let pointIndex = 0; pointIndex < cloud.pointCount; pointIndex += 1) {
    ranges[pointIndex] = usableRange(cloud, pointIndex);
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

function robustMaximumStep(first: number, second: number): number {
  const fallback = degreesToRadians(0.20);
  const finiteFirst = Number.isFinite(first) && first > 0 ? first : fallback;
  const finiteSecond = Number.isFinite(second) && second > 0 ? second : fallback;
  return Math.max(finiteFirst, finiteSecond);
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

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.max(minimum, Math.min(maximum, value));
}

function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

function invalidQuad(): QuadEvaluation {
  return { valid: false, normalX: 0, normalY: 0, normalZ: 0 };
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
