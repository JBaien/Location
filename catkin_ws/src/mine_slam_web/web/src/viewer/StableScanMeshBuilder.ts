import type { ParsedCloud } from './CloudTypes';

const TWO_PI = Math.PI * 2;

export type ScanMeshBuildReason =
  | 'ok'
  | 'empty_cloud'
  | 'missing_lidar_id'
  | 'missing_ring'
  | 'missing_azimuth';

export const QUAD_REJECT_REASONS = [
  'azimuth_gap',
  'duplicate_vertex',
  'invalid_vector',
  'invalid_range',
  'along_edge',
  'cross_edge',
  'diagonal_edge',
  'range_jump',
  'opposite_edge_angle',
  'opposite_edge_ratio',
  'triangle_degenerate',
  'triangle_area',
  'triangle_edge_ratio',
  'triangle_normal',
  'planarity',
  'invalid_quad_normal',
  'isolated_run',
  'triangle_limit'
] as const;

export type QuadRejectReason = (typeof QUAD_REJECT_REASONS)[number];
export type QuadRejectCounts = Record<QuadRejectReason, number>;

export const STRIP_BREAK_REASONS = [
  'azimuth_gap',
  'geometry_reject',
  'run_normal'
] as const;

export type StripBreakReason = (typeof STRIP_BREAK_REASONS)[number];
export type StripBreakCounts = Record<StripBreakReason, number>;

export const ISOLATED_RUN_CAUSES = [
  'azimuth_gap',
  'geometry_reject',
  'run_normal',
  'pair_end'
] as const;

export type IsolatedRunCause = (typeof ISOLATED_RUN_CAUSES)[number];
export type IsolatedRunCauseCounts = Record<IsolatedRunCause, number>;

export type RingPairStatus =
  | 'accepted'
  | 'non_adjacent'
  | 'insufficient_matches'
  | 'no_supported_run'
  | 'truncated';

export interface QuadRejectPointSample {
  pointIndex: number;
  azimuthDeg: number;
  rangeM: number;
  x: number;
  y: number;
  z: number;
}

export interface QuadRejectSample {
  reason: QuadRejectReason;
  lidarId: number;
  lowerRing: number;
  upperRing: number;
  angularGapDeg: number;
  points: {
    lower0: QuadRejectPointSample;
    upper0: QuadRejectPointSample;
    lower1: QuadRejectPointSample;
    upper1: QuadRejectPointSample;
  };
  flushCause?: StripBreakReason | 'pair_end';
  runLength?: number;
}

export interface RingDiagnostics {
  lidarId: number;
  ring: number;
  sourcePointCount: number;
  pointCount: number;
  duplicatePointCount: number;
  validRangePointCount: number;
  rangeValidity: number;
  passesMinimumPoints: boolean;
  medianAzimuthStepDeg: number;
  p90AzimuthStepDeg: number;
  maxAzimuthGapDeg: number;
  medianRangeM: number;
}

export interface RingPairDiagnostics {
  lidarId: number;
  lowerRing: number;
  upperRing: number;
  status: RingPairStatus;
  lowerPointCount: number;
  upperPointCount: number;
  matchCount: number;
  unmatchedLowerCount: number;
  unmatchedUpperCount: number;
  representativeStepDeg: number;
  matchToleranceDeg: number;
  maximumAzimuthGapDeg: number;
  candidateQuadCount: number;
  acceptedQuadCount: number;
  rejectedQuadCount: number;
  runNormalBreakCount: number;
  isolatedRunCauseCounts: IsolatedRunCauseCounts;
  rejectCounts: QuadRejectCounts;
}

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
  maxRejectSamples: number;
}

export interface ScanMeshBuildResult {
  indices: Uint32Array;
  triangleCount: number;
  sensorCount: number;
  ringCount: number;
  ringPairCount: number;
  rejectedTriangles: number;
  candidateQuadCount: number;
  evaluatedQuadCount: number;
  geometryValidQuadCount: number;
  acceptedQuadCount: number;
  rejectedQuadCount: number;
  inputPointCount: number;
  validPointCount: number;
  validRangePointCount: number;
  rangeValidity: number;
  eligiblePointCount: number;
  matchedPointCount: number;
  usedPointCount: number;
  eligiblePointRelations: number;
  matchedPointRelations: number;
  matchCoverage: number;
  pointCoverage: number;
  quadCoverage: number;
  truncatedByMaxTriangles: boolean;
  rejectCounts: QuadRejectCounts;
  stripBreakCounts: StripBreakCounts;
  isolatedRunCauseCounts: IsolatedRunCauseCounts;
  rejectSamples: QuadRejectSample[];
  rings: RingDiagnostics[];
  ringPairs: RingPairDiagnostics[];
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
  minRunQuads: 2,

  // Keep detailed examples bounded. Counters remain complete, while the sample
  // cap prevents diagnostic data from becoming a new 10 Hz performance issue.
  maxRejectSamples: 32
};

interface RingSequence {
  ring: number;
  sourcePointCount: number;
  validRangePointCount: number;
  indices: number[];
  medianStep: number;
  p90Step: number;
  maxGap: number;
  medianRange: number;
}

interface RingMatch {
  lowerPoint: number;
  upperPoint: number;
  angle: number;
}

interface SensorRings {
  rings: Map<number, number[]>;
}

interface QuadVertices {
  lower0: number;
  upper0: number;
  lower1: number;
  upper1: number;
  angularGap: number;
}

interface QuadCandidate extends QuadVertices {
  normalX: number;
  normalY: number;
  normalZ: number;
}

type QuadEvaluation =
  | {
      valid: true;
      normalX: number;
      normalY: number;
      normalZ: number;
    }
  | {
      valid: false;
      reason: Exclude<QuadRejectReason, 'azimuth_gap' | 'isolated_run' | 'triangle_limit'>;
    };

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
  if (cloud.pointCount < 4) return emptyResult('empty_cloud', cloud.pointCount);
  if (!cloud.hasLidarId) return emptyResult('missing_lidar_id', cloud.pointCount);
  if (!cloud.hasRing) return emptyResult('missing_ring', cloud.pointCount);
  if (!cloud.hasAzimuth) return emptyResult('missing_azimuth', cloud.pointCount);

  const sensors = collectSensorRings(cloud);
  const maxTriangles = finiteNonNegativeInteger(
    options.maxTriangles,
    DEFAULT_STABLE_SCAN_MESH_OPTIONS.maxTriangles
  );
  const output = new Uint32Array(maxTriangles * 3);
  const ranges = buildRanges(cloud);
  const rejectCounts = createRejectCounts();
  const stripBreakCounts = createStripBreakCounts();
  const isolatedRunCauseCounts = createIsolatedRunCauseCounts();
  const rejectSamples: QuadRejectSample[] = [];
  const rings: RingDiagnostics[] = [];
  const ringPairs: RingPairDiagnostics[] = [];
  const eligiblePointMask = new Uint8Array(cloud.pointCount);
  const matchedPointMask = new Uint8Array(cloud.pointCount);
  const usedPointMask = new Uint8Array(cloud.pointCount);
  const minimumRunQuads = Math.max(
    1,
    finiteNonNegativeInteger(
      options.minRunQuads,
      DEFAULT_STABLE_SCAN_MESH_OPTIONS.minRunQuads
    )
  );
  const minimumPointsPerRing = Math.max(
    1,
    finiteNonNegativeInteger(
      options.minPointsPerRing,
      DEFAULT_STABLE_SCAN_MESH_OPTIONS.minPointsPerRing
    )
  );
  const maximumRejectSamples = finiteNonNegativeInteger(
    options.maxRejectSamples,
    DEFAULT_STABLE_SCAN_MESH_OPTIONS.maxRejectSamples
  );

  let outputCount = 0;
  let ringCount = 0;
  let ringPairCount = 0;
  let candidateQuadCount = 0;
  let evaluatedQuadCount = 0;
  let geometryValidQuadCount = 0;
  let acceptedQuadCount = 0;
  let rejectedQuadCount = 0;
  let eligiblePointRelations = 0;
  let matchedPointRelations = 0;
  let truncatedByMaxTriangles = false;

  const appendQuad = (candidate: QuadCandidate): boolean => {
    if (outputCount + 6 > output.length) return false;
    output[outputCount] = candidate.lower0;
    output[outputCount + 1] = candidate.upper0;
    output[outputCount + 2] = candidate.lower1;
    output[outputCount + 3] = candidate.lower1;
    output[outputCount + 4] = candidate.upper0;
    output[outputCount + 5] = candidate.upper1;
    outputCount += 6;
    return true;
  };

  const addRejectSample = (
    reason: QuadRejectReason,
    lidarId: number,
    lowerRing: number,
    upperRing: number,
    candidate: QuadVertices,
    flushCause?: StripBreakReason | 'pair_end',
    runLength?: number
  ): void => {
    if (maximumRejectSamples === 0) return;
    let replaceIndex = rejectSamples.length;
    if (rejectSamples.length >= maximumRejectSamples) {
      // Preserve at least one example of a newly observed reason when possible,
      // rather than filling the bounded sample with the first frequent failure.
      if (rejectSamples.some((existing) => existing.reason === reason)) return;
      const frequencies = new Map<QuadRejectReason, number>();
      for (const existing of rejectSamples) {
        frequencies.set(existing.reason, (frequencies.get(existing.reason) ?? 0) + 1);
      }
      replaceIndex = rejectSamples.findIndex(
        (existing) => (frequencies.get(existing.reason) ?? 0) > 1
      );
      if (replaceIndex < 0) return;
    }

    const sample = createRejectSample(
      reason,
      lidarId,
      lowerRing,
      upperRing,
      candidate,
      cloud,
      ranges,
      flushCause,
      runLength
    );
    if (replaceIndex === rejectSamples.length) rejectSamples.push(sample);
    else rejectSamples[replaceIndex] = sample;
  };

  const recordReject = (
    pair: RingPairDiagnostics,
    reason: QuadRejectReason,
    candidate: QuadVertices,
    flushCause?: StripBreakReason | 'pair_end',
    runLength?: number
  ): void => {
    rejectedQuadCount += 1;
    rejectCounts[reason] += 1;
    pair.rejectedQuadCount += 1;
    pair.rejectCounts[reason] += 1;
    if (reason === 'isolated_run' && flushCause) {
      isolatedRunCauseCounts[flushCause] += 1;
      pair.isolatedRunCauseCounts[flushCause] += 1;
    }
    addRejectSample(
      reason,
      pair.lidarId,
      pair.lowerRing,
      pair.upperRing,
      candidate,
      flushCause,
      runLength
    );
  };

  for (const [lidarId, sensor] of sensors.entries()) {
    const preparedSequences = Array.from(sensor.rings.entries())
      .map(([ring, indices]) => prepareRingSequence(ring, indices, cloud))
      .sort((left, right) => left.ring - right.ring);
    for (const sequence of preparedSequences) {
      rings.push({
        lidarId,
        ring: sequence.ring,
        sourcePointCount: sequence.sourcePointCount,
        pointCount: sequence.indices.length,
        duplicatePointCount: sequence.sourcePointCount - sequence.indices.length,
        validRangePointCount: sequence.validRangePointCount,
        rangeValidity: safeRatio(sequence.validRangePointCount, sequence.sourcePointCount),
        passesMinimumPoints: sequence.indices.length >= minimumPointsPerRing,
        medianAzimuthStepDeg: radiansToDegrees(sequence.medianStep),
        p90AzimuthStepDeg: radiansToDegrees(sequence.p90Step),
        maxAzimuthGapDeg: radiansToDegrees(sequence.maxGap),
        medianRangeM: sequence.medianRange
      });
    }
    const sequences = preparedSequences.filter(
      (sequence) => sequence.indices.length >= minimumPointsPerRing
    );
    ringCount += sequences.length;

    for (let sequenceIndex = 0; sequenceIndex + 1 < sequences.length; sequenceIndex += 1) {
      const lower = sequences[sequenceIndex];
      const upper = sequences[sequenceIndex + 1];
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

      // Keep skipped pairs visible in diagnostics. This is especially important
      // while raw ring id order has not yet been verified against beam elevation.
      if (upper.ring - lower.ring !== 1) {
        ringPairs.push({
          lidarId,
          lowerRing: lower.ring,
          upperRing: upper.ring,
          status: 'non_adjacent',
          lowerPointCount: lower.indices.length,
          upperPointCount: upper.indices.length,
          matchCount: 0,
          unmatchedLowerCount: lower.indices.length,
          unmatchedUpperCount: upper.indices.length,
          representativeStepDeg: radiansToDegrees(representativeStep),
          matchToleranceDeg: radiansToDegrees(matchTolerance),
          maximumAzimuthGapDeg: radiansToDegrees(maximumAzimuthGap),
          candidateQuadCount: 0,
          acceptedQuadCount: 0,
          rejectedQuadCount: 0,
          runNormalBreakCount: 0,
          isolatedRunCauseCounts: createIsolatedRunCauseCounts(),
          rejectCounts: createRejectCounts()
        });
        continue;
      }

      markPoints(eligiblePointMask, lower.indices);
      markPoints(eligiblePointMask, upper.indices);
      eligiblePointRelations += lower.indices.length + upper.indices.length;

      const matches = matchRingSequences(lower, upper, cloud.azimuths, matchTolerance);
      matchedPointRelations += matches.length * 2;
      for (const match of matches) {
        matchedPointMask[match.lowerPoint] = 1;
        matchedPointMask[match.upperPoint] = 1;
      }

      const pair: RingPairDiagnostics = {
        lidarId,
        lowerRing: lower.ring,
        upperRing: upper.ring,
        status: 'no_supported_run',
        lowerPointCount: lower.indices.length,
        upperPointCount: upper.indices.length,
        matchCount: matches.length,
        unmatchedLowerCount: Math.max(0, lower.indices.length - matches.length),
        unmatchedUpperCount: Math.max(0, upper.indices.length - matches.length),
        representativeStepDeg: radiansToDegrees(representativeStep),
        matchToleranceDeg: radiansToDegrees(matchTolerance),
        maximumAzimuthGapDeg: radiansToDegrees(maximumAzimuthGap),
        candidateQuadCount: 0,
        acceptedQuadCount: 0,
        rejectedQuadCount: 0,
        runNormalBreakCount: 0,
        isolatedRunCauseCounts: createIsolatedRunCauseCounts(),
        rejectCounts: createRejectCounts()
      };
      ringPairs.push(pair);
      if (matches.length < 2) {
        pair.status = 'insufficient_matches';
        continue;
      }

      let run: QuadCandidate[] = [];
      let pairAccepted = false;

      const flushRun = (cause: StripBreakReason | 'pair_end'): void => {
        if (run.length < minimumRunQuads) {
          const runLength = run.length;
          for (const candidate of run) {
            recordReject(pair, 'isolated_run', candidate, cause, runLength);
          }
          run = [];
          return;
        }
        for (let runIndex = 0; runIndex < run.length; runIndex += 1) {
          const candidate = run[runIndex];
          if (!appendQuad(candidate)) {
            truncatedByMaxTriangles = true;
            pair.status = 'truncated';
            for (let rejectedIndex = runIndex; rejectedIndex < run.length; rejectedIndex += 1) {
              recordReject(pair, 'triangle_limit', run[rejectedIndex], cause);
            }
            run = [];
            return;
          }
          acceptedQuadCount += 1;
          pair.acceptedQuadCount += 1;
          pairAccepted = true;
          usedPointMask[candidate.lower0] = 1;
          usedPointMask[candidate.upper0] = 1;
          usedPointMask[candidate.lower1] = 1;
          usedPointMask[candidate.upper1] = 1;
        }
        run = [];
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
        const vertices: QuadVertices = {
          lower0: previous.lowerPoint,
          upper0: previous.upperPoint,
          lower1: current.lowerPoint,
          upper1: current.upperPoint,
          angularGap
        };
        candidateQuadCount += 1;
        pair.candidateQuadCount += 1;

        if (
          !Number.isFinite(angularGap) ||
          angularGap <= 0 ||
          angularGap > maximumAzimuthGap
        ) {
          recordReject(pair, 'azimuth_gap', vertices);
          stripBreakCounts.azimuth_gap += 1;
          flushRun('azimuth_gap');
          continue;
        }

        evaluatedQuadCount += 1;
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
          recordReject(pair, evaluation.reason, vertices);
          stripBreakCounts.geometry_reject += 1;
          flushRun('geometry_reject');
          continue;
        }
        geometryValidQuadCount += 1;

        const candidate: QuadCandidate = {
          ...vertices,
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
          stripBreakCounts.run_normal += 1;
          pair.runNormalBreakCount += 1;
          flushRun('run_normal');
        }
        run.push(candidate);
      }

      flushRun('pair_end');
      if (pairAccepted) {
        ringPairCount += 1;
        if (pair.status !== 'truncated') pair.status = 'accepted';
      }
    }
  }

  const validPointCount = countCollectedPoints(sensors);
  const validRangePointCount = countValidRangePoints(sensors, ranges);
  const eligiblePointCount = countMarkedPoints(eligiblePointMask);
  const matchedPointCount = countMarkedPoints(matchedPointMask);
  const usedPointCount = countMarkedPoints(usedPointMask);

  return {
    indices: output.slice(0, outputCount),
    triangleCount: outputCount / 3,
    sensorCount: sensors.size,
    ringCount,
    ringPairCount,
    rejectedTriangles: rejectedQuadCount * 2,
    candidateQuadCount,
    evaluatedQuadCount,
    geometryValidQuadCount,
    acceptedQuadCount,
    rejectedQuadCount,
    inputPointCount: cloud.pointCount,
    validPointCount,
    validRangePointCount,
    rangeValidity: safeRatio(validRangePointCount, validPointCount),
    eligiblePointCount,
    matchedPointCount,
    usedPointCount,
    eligiblePointRelations,
    matchedPointRelations,
    matchCoverage: safeRatio(matchedPointRelations, eligiblePointRelations),
    pointCoverage: safeRatio(usedPointCount, eligiblePointCount),
    quadCoverage: safeRatio(acceptedQuadCount, candidateQuadCount),
    truncatedByMaxTriangles,
    rejectCounts,
    stripBreakCounts,
    isolatedRunCauseCounts,
    rejectSamples,
    rings,
    ringPairs,
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
    const currentRange = usableRange(cloud, pointIndex);
    const previousRange = usableRange(cloud, previousIndex);
    const currentRangeValid = Number.isFinite(currentRange) && currentRange > 0;
    const previousRangeValid = Number.isFinite(previousRange) && previousRange > 0;
    if (
      (currentRangeValid && !previousRangeValid) ||
      (currentRangeValid && previousRangeValid && currentRange < previousRange)
    ) {
      indices[indices.length - 1] = pointIndex;
    }
  }

  const stepStats = estimateAzimuthStepStats(indices, cloud.azimuths);
  const sampledRanges: number[] = [];
  const rangeStride = Math.max(1, Math.floor(indices.length / 256));
  for (let order = 0; order < indices.length; order += rangeStride) {
    const range = usableRange(cloud, indices[order]);
    if (Number.isFinite(range) && range > 0) sampledRanges.push(range);
  }

  return {
    ring,
    sourcePointCount: sourceIndices.length,
    validRangePointCount: sourceIndices.reduce((count, pointIndex) => {
      const range = usableRange(cloud, pointIndex);
      return count + (Number.isFinite(range) && range > 0 ? 1 : 0);
    }, 0),
    indices,
    medianStep: stepStats.median,
    p90Step: stepStats.p90,
    maxGap: stepStats.maximum,
    medianRange: percentile(sampledRanges, 0.5)
  };
}

interface AzimuthStepStats {
  median: number;
  p90: number;
  maximum: number;
}

function estimateAzimuthStepStats(
  indices: number[],
  azimuths: Float32Array
): AzimuthStepStats {
  const nativeDeltas: number[] = [];
  const allDeltas: number[] = [];
  const maximumSampleGap = degreesToRadians(2.0);
  for (let order = 1; order < indices.length; order += 1) {
    const delta = positiveAngleDelta(
      azimuths[indices[order - 1]],
      azimuths[indices[order]]
    );
    if (Number.isFinite(delta) && delta > 1e-6) {
      allDeltas.push(delta);
      if (delta <= maximumSampleGap) nativeDeltas.push(delta);
    }
  }
  if (indices.length > 1) {
    const wrapDelta = positiveAngleDelta(
      azimuths[indices[indices.length - 1]],
      azimuths[indices[0]]
    );
    if (Number.isFinite(wrapDelta) && wrapDelta > 1e-6) allDeltas.push(wrapDelta);
  }

  const fallback = degreesToRadians(0.20);
  return {
    median: percentile(nativeDeltas, 0.5, fallback),
    p90: percentile(nativeDeltas, 0.9, fallback),
    maximum: allDeltas.length > 0 ? Math.max(...allDeltas) : 0
  };
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
    return invalidQuad('duplicate_vertex');
  }

  const alongLower = vectorBetween(lower0, lower1, positions);
  const alongUpper = vectorBetween(upper0, upper1, positions);
  const crossStart = vectorBetween(lower0, upper0, positions);
  const crossEnd = vectorBetween(lower1, upper1, positions);
  const diagonalA = vectorBetween(lower0, upper1, positions);
  const diagonalB = vectorBetween(upper0, lower1, positions);
  const vectors = [alongLower, alongUpper, crossStart, crossEnd, diagonalA, diagonalB];
  if (vectors.some((vector) => !vector.valid || vector.length <= 1e-6)) {
    return invalidQuad('invalid_vector');
  }

  const quadRanges = [ranges[lower0], ranges[upper0], ranges[lower1], ranges[upper1]];
  if (quadRanges.some((range) => !Number.isFinite(range) || range <= 0)) {
    return invalidQuad('invalid_range');
  }
  const minimumRange = Math.max(0.01, Math.min(...quadRanges));
  const maximumRange = Math.max(...quadRanges);

  const allowedAlongEdge = Math.min(
    options.maxAlongEdgeM,
    options.baseAlongEdgeM + maximumRange * angularGap * options.angularEdgeScale
  );
  if (Math.max(alongLower.length, alongUpper.length) > allowedAlongEdge) {
    return invalidQuad('along_edge');
  }
  if (Math.max(crossStart.length, crossEnd.length) > options.maxCrossEdgeM) {
    return invalidQuad('cross_edge');
  }
  if (Math.max(diagonalA.length, diagonalB.length) > options.maxDiagonalEdgeM) {
    return invalidQuad('diagonal_edge');
  }

  const allowedAlongRangeJump = Math.min(
    options.maxAlongRangeJumpM,
    options.baseAlongRangeJumpM + minimumRange * options.alongRangeJumpRatio
  );
  if (
    Math.abs(ranges[lower1] - ranges[lower0]) > allowedAlongRangeJump ||
    Math.abs(ranges[upper1] - ranges[upper0]) > allowedAlongRangeJump
  ) {
    return invalidQuad('range_jump');
  }

  if (
    !vectorsAgree(alongLower, alongUpper, options.maxOppositeEdgeAngleDeg) ||
    !vectorsAgree(crossStart, crossEnd, options.maxOppositeEdgeAngleDeg)
  ) {
    return invalidQuad('opposite_edge_angle');
  }
  if (
    edgeLengthRatio(alongLower.length, alongUpper.length) >
      options.maxOppositeEdgeLengthRatio ||
    edgeLengthRatio(crossStart.length, crossEnd.length) >
      options.maxOppositeEdgeLengthRatio
  ) {
    return invalidQuad('opposite_edge_ratio');
  }

  const first = triangleMetrics(lower0, upper0, lower1, positions);
  const second = triangleMetrics(lower1, upper0, upper1, positions);
  if (!first.valid || !second.valid) return invalidQuad('triangle_degenerate');
  if (first.area < options.minTriangleAreaM2 || second.area < options.minTriangleAreaM2) {
    return invalidQuad('triangle_area');
  }
  if (first.edgeRatio > options.maxEdgeRatio || second.edgeRatio > options.maxEdgeRatio) {
    return invalidQuad('triangle_edge_ratio');
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
    return invalidQuad('triangle_normal');
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
    return invalidQuad('planarity');
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
  if (!Number.isFinite(normalLength) || normalLength <= 1e-9) {
    return invalidQuad('invalid_quad_normal');
  }

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
  if (cloud.hasRange) return cloud.ranges[pointIndex];
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

function createRejectCounts(): QuadRejectCounts {
  const counts = {} as QuadRejectCounts;
  for (const reason of QUAD_REJECT_REASONS) counts[reason] = 0;
  return counts;
}

function createStripBreakCounts(): StripBreakCounts {
  const counts = {} as StripBreakCounts;
  for (const reason of STRIP_BREAK_REASONS) counts[reason] = 0;
  return counts;
}

function createIsolatedRunCauseCounts(): IsolatedRunCauseCounts {
  const counts = {} as IsolatedRunCauseCounts;
  for (const cause of ISOLATED_RUN_CAUSES) counts[cause] = 0;
  return counts;
}

function createRejectSample(
  reason: QuadRejectReason,
  lidarId: number,
  lowerRing: number,
  upperRing: number,
  candidate: QuadVertices,
  cloud: ParsedCloud,
  ranges: Float32Array,
  flushCause?: StripBreakReason | 'pair_end',
  runLength?: number
): QuadRejectSample {
  const point = (pointIndex: number): QuadRejectPointSample => {
    const offset = pointIndex * 3;
    return {
      pointIndex,
      azimuthDeg: radiansToDegrees(cloud.azimuths[pointIndex]),
      rangeM: ranges[pointIndex],
      x: cloud.positions[offset],
      y: cloud.positions[offset + 1],
      z: cloud.positions[offset + 2]
    };
  };
  return {
    reason,
    lidarId,
    lowerRing,
    upperRing,
    angularGapDeg: radiansToDegrees(candidate.angularGap),
    points: {
      lower0: point(candidate.lower0),
      upper0: point(candidate.upper0),
      lower1: point(candidate.lower1),
      upper1: point(candidate.upper1)
    },
    ...(flushCause ? { flushCause } : {}),
    ...(runLength === undefined ? {} : { runLength })
  };
}

function countCollectedPoints(sensors: Map<number, SensorRings>): number {
  let count = 0;
  for (const sensor of sensors.values()) {
    for (const indices of sensor.rings.values()) count += indices.length;
  }
  return count;
}

function countValidRangePoints(
  sensors: Map<number, SensorRings>,
  ranges: Float32Array
): number {
  let count = 0;
  for (const sensor of sensors.values()) {
    for (const indices of sensor.rings.values()) {
      for (const pointIndex of indices) {
        if (Number.isFinite(ranges[pointIndex]) && ranges[pointIndex] > 0) count += 1;
      }
    }
  }
  return count;
}

function markPoints(mask: Uint8Array, indices: number[]): void {
  for (const pointIndex of indices) mask[pointIndex] = 1;
}

function countMarkedPoints(mask: Uint8Array): number {
  let count = 0;
  for (const value of mask) count += value;
  return count;
}

function safeRatio(numerator: number, denominator: number): number {
  if (denominator <= 0) return 0;
  return Math.max(0, Math.min(1, numerator / denominator));
}

function finiteNonNegativeInteger(value: number, fallback: number): number {
  if (!Number.isFinite(value)) return Math.max(0, Math.floor(fallback));
  return Math.max(0, Math.floor(value));
}

function percentile(values: number[], quantile: number, fallback = 0): number {
  if (values.length === 0) return fallback;
  values.sort((left, right) => left - right);
  const index = Math.min(
    values.length - 1,
    Math.max(0, Math.floor((values.length - 1) * quantile))
  );
  return values[index];
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

function radiansToDegrees(radians: number): number {
  return (radians * 180) / Math.PI;
}

function invalidQuad(
  reason: Exclude<QuadRejectReason, 'azimuth_gap' | 'isolated_run' | 'triangle_limit'>
): QuadEvaluation {
  return { valid: false, reason };
}

function emptyResult(
  reason: Exclude<ScanMeshBuildReason, 'ok'>,
  inputPointCount = 0
): ScanMeshBuildResult {
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
    inputPointCount,
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
    rejectCounts: createRejectCounts(),
    stripBreakCounts: createStripBreakCounts(),
    isolatedRunCauseCounts: createIsolatedRunCauseCounts(),
    rejectSamples: [],
    rings: [],
    ringPairs: [],
    reason
  };
}
