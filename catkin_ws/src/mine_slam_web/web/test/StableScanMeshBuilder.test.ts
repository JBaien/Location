import assert from 'node:assert/strict';
import test from 'node:test';

import type { ParsedCloud } from '../src/viewer/CloudTypes';
import {
  buildStableScanMesh,
  DEFAULT_STABLE_SCAN_MESH_OPTIONS,
  QUAD_REJECT_REASONS,
  type ScanMeshBuildResult,
  type StableScanMeshBuildOptions
} from '../src/viewer/StableScanMeshBuilder';

function makePlanarCloud(
  ringCount: number,
  columnRanges: number[]
): ParsedCloud {
  const pointCount = ringCount * columnRanges.length;
  const positions = new Float32Array(pointCount * 3);
  const intensities = new Float32Array(pointCount);
  const times = new Float32Array(pointCount);
  const azimuths = new Float32Array(pointCount);
  const ranges = new Float32Array(pointCount);
  const rings = new Uint16Array(pointCount);
  const lidarIds = new Uint8Array(pointCount);
  const classIds = new Uint8Array(pointCount);

  let pointIndex = 0;
  for (let ring = 0; ring < ringCount; ring += 1) {
    for (let column = 0; column < columnRanges.length; column += 1) {
      const offset = pointIndex * 3;
      positions[offset] = column * 0.05;
      positions[offset + 1] = ring * 0.2;
      positions[offset + 2] = 0;
      intensities[pointIndex] = 50;
      azimuths[pointIndex] = (column * 0.2 * Math.PI) / 180;
      ranges[pointIndex] = columnRanges[column];
      rings[pointIndex] = ring;
      lidarIds[pointIndex] = 0;
      classIds[pointIndex] = 2;
      pointIndex += 1;
    }
  }

  return {
    protocolVersion: 3,
    cloudType: 1,
    stampNs: 1n,
    pointCount,
    fieldsMask: 0xf7,
    positions,
    intensities,
    times,
    azimuths,
    ranges,
    rings,
    lidarIds,
    classIds,
    hasLidarId: true,
    hasRing: true,
    hasTime: true,
    hasAzimuth: true,
    hasRange: true
  };
}

function build(
  cloud: ParsedCloud,
  overrides: Partial<StableScanMeshBuildOptions> = {}
): ScanMeshBuildResult {
  return buildStableScanMesh(cloud, {
    ...DEFAULT_STABLE_SCAN_MESH_OPTIONS,
    minPointsPerRing: 2,
    ...overrides
  });
}

function assertAccounting(result: ScanMeshBuildResult): void {
  assert.equal(result.triangleCount, result.acceptedQuadCount * 2);
  assert.equal(result.indices.length, result.acceptedQuadCount * 6);
  assert.equal(
    result.candidateQuadCount,
    result.acceptedQuadCount + result.rejectedQuadCount
  );
  assert.equal(result.rejectedTriangles, result.rejectedQuadCount * 2);
  assert.equal(
    QUAD_REJECT_REASONS.reduce(
      (total, reason) => total + result.rejectCounts[reason],
      0
    ),
    result.rejectedQuadCount
  );
  assert.equal(
    result.candidateQuadCount,
    result.evaluatedQuadCount + result.rejectCounts.azimuth_gap
  );
  const geometryRejectCount = QUAD_REJECT_REASONS
    .filter(
      (reason) =>
        reason !== 'azimuth_gap' &&
        reason !== 'isolated_run' &&
        reason !== 'triangle_limit'
    )
    .reduce((total, reason) => total + result.rejectCounts[reason], 0);
  assert.equal(
    result.evaluatedQuadCount,
    result.geometryValidQuadCount + geometryRejectCount
  );
  assert.equal(
    result.geometryValidQuadCount,
    result.acceptedQuadCount +
      result.rejectCounts.isolated_run +
      result.rejectCounts.triangle_limit
  );
  assert.ok(result.usedPointCount <= result.matchedPointCount);
  assert.ok(result.matchedPointCount <= result.eligiblePointCount);
  assert.ok(result.eligiblePointCount <= result.validPointCount);
  assert.ok(result.validPointCount <= result.inputPointCount);

  let pairCandidates = 0;
  let pairAccepted = 0;
  let pairRejected = 0;
  let pairRunNormalBreaks = 0;
  let pairsWithOutput = 0;
  const pairReasonCounts = Object.fromEntries(
    QUAD_REJECT_REASONS.map((reason) => [reason, 0])
  ) as Record<(typeof QUAD_REJECT_REASONS)[number], number>;
  for (const pair of result.ringPairs) {
    const pairRejectSum = QUAD_REJECT_REASONS.reduce(
      (total, reason) => total + pair.rejectCounts[reason],
      0
    );
    assert.equal(pair.candidateQuadCount, pair.acceptedQuadCount + pair.rejectedQuadCount);
    assert.equal(pairRejectSum, pair.rejectedQuadCount);
    pairCandidates += pair.candidateQuadCount;
    pairAccepted += pair.acceptedQuadCount;
    pairRejected += pair.rejectedQuadCount;
    pairRunNormalBreaks += pair.runNormalBreakCount;
    if (pair.acceptedQuadCount > 0) pairsWithOutput += 1;
    for (const reason of QUAD_REJECT_REASONS) {
      pairReasonCounts[reason] += pair.rejectCounts[reason];
    }
  }
  assert.equal(pairCandidates, result.candidateQuadCount);
  assert.equal(pairAccepted, result.acceptedQuadCount);
  assert.equal(pairRejected, result.rejectedQuadCount);
  assert.equal(pairRunNormalBreaks, result.stripBreakCounts.run_normal);
  assert.equal(pairsWithOutput, result.ringPairCount);
  assert.equal(
    Object.values(result.isolatedRunCauseCounts).reduce(
      (total, count) => total + count,
      0
    ),
    result.rejectCounts.isolated_run
  );
  for (const reason of QUAD_REJECT_REASONS) {
    assert.equal(pairReasonCounts[reason], result.rejectCounts[reason]);
  }
  for (const coverage of [
    result.matchCoverage,
    result.pointCoverage,
    result.quadCoverage
  ]) {
    assert.ok(Number.isFinite(coverage));
    assert.ok(coverage >= 0 && coverage <= 1);
  }
}

test('clean adjacent rings produce complete coverage without double-counting points', () => {
  const result = build(makePlanarCloud(3, [10, 10, 10, 10, 10]));

  assert.equal(result.candidateQuadCount, 8);
  assert.equal(result.acceptedQuadCount, 8);
  assert.equal(result.rejectedQuadCount, 0);
  assert.equal(result.triangleCount, 16);
  assert.equal(result.eligiblePointCount, 15);
  assert.equal(result.matchedPointCount, 15);
  assert.equal(result.usedPointCount, 15);
  assert.equal(result.matchCoverage, 1);
  assert.equal(result.pointCoverage, 1);
  assert.equal(result.quadCoverage, 1);
  assert.equal(result.ringPairCount, 2);
  assertAccounting(result);
});

test('a rejected range transition exposes the short-run hole amplification', () => {
  const cloud = makePlanarCloud(2, [10, 10, 20, 20, 20]);
  const strictRun = build(cloud, { minRunQuads: 2 });

  assert.equal(strictRun.candidateQuadCount, 4);
  assert.equal(strictRun.evaluatedQuadCount, 4);
  assert.equal(strictRun.geometryValidQuadCount, 3);
  assert.equal(strictRun.acceptedQuadCount, 2);
  assert.equal(strictRun.rejectedQuadCount, 2);
  assert.equal(strictRun.rejectCounts.range_jump, 1);
  assert.equal(strictRun.rejectCounts.isolated_run, 1);
  assert.equal(strictRun.quadCoverage, 0.5);
  assert.equal(strictRun.matchCoverage, 1);
  assert.equal(strictRun.pointCoverage, 0.6);
  assert.deepEqual(
    new Set(strictRun.rejectSamples.map((sample) => sample.reason)),
    new Set(['range_jump', 'isolated_run'])
  );
  assertAccounting(strictRun);

  const relaxedRun = build(cloud, { minRunQuads: 1 });
  assert.equal(relaxedRun.acceptedQuadCount, 3);
  assert.equal(relaxedRun.rejectedQuadCount, 1);
  assert.equal(relaxedRun.rejectCounts.range_jump, 1);
  assert.equal(relaxedRun.rejectCounts.isolated_run, 0);
  assert.equal(relaxedRun.quadCoverage, 0.75);
  assert.equal(relaxedRun.pointCoverage, 1);
  assertAccounting(relaxedRun);
});

test('triangle capacity remains atomic and reports all processed truncation rejects', () => {
  const result = build(makePlanarCloud(2, [10, 10, 10, 10, 10]), {
    maxTriangles: 2
  });

  assert.equal(result.triangleCount, 2);
  assert.equal(result.acceptedQuadCount, 1);
  assert.equal(result.rejectedQuadCount, 3);
  assert.equal(result.rejectCounts.triangle_limit, 3);
  assert.equal(result.truncatedByMaxTriangles, true);
  assert.equal(result.indices.length, 6);
  assert.equal(result.ringPairCount, 1);
  assertAccounting(result);
});

test('reject detail samples stay bounded without reducing complete counters', () => {
  const cloud = makePlanarCloud(2, [10, 20, 10, 20, 10, 20]);
  const bounded = build(cloud, { maxRejectSamples: 2 });
  assert.equal(bounded.candidateQuadCount, 5);
  assert.equal(bounded.rejectCounts.range_jump, 5);
  assert.equal(bounded.rejectSamples.length, 2);
  assertAccounting(bounded);

  const countersOnly = build(cloud, { maxRejectSamples: 0 });
  assert.equal(countersOnly.rejectCounts.range_jump, 5);
  assert.equal(countersOnly.rejectSamples.length, 0);
  assertAccounting(countersOnly);
});

test('run normal is a strip break while only the unsupported quad is rejected', () => {
  const cloud = makePlanarCloud(2, [10, 10, 10, 10]);
  const trace = [
    [0, 0],
    [0.05, 0],
    [0.05 + 0.05 * Math.cos((20 * Math.PI) / 180), 0.05 * Math.sin((20 * Math.PI) / 180)],
    [0.05 + 0.10 * Math.cos((20 * Math.PI) / 180), 0.10 * Math.sin((20 * Math.PI) / 180)]
  ];
  for (let ring = 0; ring < 2; ring += 1) {
    for (let column = 0; column < trace.length; column += 1) {
      const pointIndex = ring * trace.length + column;
      cloud.positions[pointIndex * 3] = trace[column][0];
      cloud.positions[pointIndex * 3 + 1] = ring * 0.2;
      cloud.positions[pointIndex * 3 + 2] = trace[column][1];
    }
  }

  const result = build(cloud, { maxRunNormalAngleDeg: 5, minRunQuads: 2 });
  assert.equal(result.candidateQuadCount, 3);
  assert.equal(result.evaluatedQuadCount, 3);
  assert.equal(result.geometryValidQuadCount, 3);
  assert.equal(result.acceptedQuadCount, 2);
  assert.equal(result.rejectCounts.isolated_run, 1);
  assert.equal(result.stripBreakCounts.run_normal, 1);
  assert.equal(result.ringPairs[0].runNormalBreakCount, 1);
  assert.equal(result.isolatedRunCauseCounts.run_normal, 1);
  assert.equal(result.rejectSamples[0].flushCause, 'run_normal');
  assert.equal(result.rejectSamples[0].runLength, 1);
  assertAccounting(result);

  const truncated = build(cloud, {
    maxRunNormalAngleDeg: 5,
    minRunQuads: 2,
    maxTriangles: 2
  });
  assert.equal(truncated.acceptedQuadCount, 1);
  assert.equal(truncated.rejectCounts.triangle_limit, 1);
  assert.equal(truncated.rejectCounts.isolated_run, 1);
  assertAccounting(truncated);
});

test('invalid source ranges and non-adjacent rings are diagnosed explicitly', () => {
  const invalidRange = build(makePlanarCloud(2, [10, 0, 10]));
  assert.equal(invalidRange.rejectCounts.invalid_range, 2);
  assertAccounting(invalidRange);

  const nonAdjacentCloud = makePlanarCloud(2, [10, 10, 10]);
  for (let pointIndex = 3; pointIndex < nonAdjacentCloud.pointCount; pointIndex += 1) {
    nonAdjacentCloud.rings[pointIndex] = 2;
  }
  const nonAdjacent = build(nonAdjacentCloud);
  assert.equal(nonAdjacent.ringPairs.length, 1);
  assert.equal(nonAdjacent.ringPairs[0].status, 'non_adjacent');
  assert.equal(nonAdjacent.eligiblePointCount, 0);
  assertAccounting(nonAdjacent);
});

test('duplicate azimuth selection prefers a valid source range before nearest return', () => {
  const cloud = makePlanarCloud(2, [10, 10, 10]);
  const originalPointCount = cloud.pointCount;
  const positions = new Float32Array((originalPointCount + 2) * 3);
  positions.set(cloud.positions);
  const azimuths = new Float32Array(originalPointCount + 2);
  azimuths.set(cloud.azimuths);
  const ranges = new Float32Array(originalPointCount + 2);
  ranges.set(cloud.ranges);
  const rings = new Uint16Array(originalPointCount + 2);
  rings.set(cloud.rings);
  const lidarIds = new Uint8Array(originalPointCount + 2);
  lidarIds.set(cloud.lidarIds);

  for (let ring = 0; ring < 2; ring += 1) {
    const sourceIndex = ring * 3 + 1;
    const duplicateIndex = originalPointCount + ring;
    positions.set(cloud.positions.subarray(sourceIndex * 3, sourceIndex * 3 + 3), duplicateIndex * 3);
    azimuths[duplicateIndex] = cloud.azimuths[sourceIndex];
    ranges[duplicateIndex] = 0;
    rings[duplicateIndex] = ring;
  }
  const extended: ParsedCloud = {
    ...cloud,
    pointCount: originalPointCount + 2,
    positions,
    azimuths,
    ranges,
    rings,
    lidarIds,
    intensities: new Float32Array(originalPointCount + 2),
    times: new Float32Array(originalPointCount + 2),
    classIds: new Uint8Array(originalPointCount + 2)
  };
  const result = build(extended);
  assert.equal(result.acceptedQuadCount, 2);
  assert.equal(result.rejectCounts.invalid_range, 0);
  assert.equal(result.validRangePointCount, originalPointCount);
  assert.equal(result.rings[0].duplicatePointCount, 1);
  assertAccounting(result);
});

test('empty results keep a stable zero-filled diagnostic schema', () => {
  const result = build({ ...makePlanarCloud(1, [10]), pointCount: 0 });

  assert.equal(result.reason, 'empty_cloud');
  assert.equal(result.candidateQuadCount, 0);
  assert.equal(result.rejectSamples.length, 0);
  assert.equal(result.rings.length, 0);
  assert.equal(result.ringPairs.length, 0);
  for (const reason of QUAD_REJECT_REASONS) {
    assert.equal(result.rejectCounts[reason], 0);
  }
  assertAccounting(result);
});

test(
  'connects a local scan strip across the circular azimuth seam',
  { todo: 'circular sequence ordering is a separate P1 mesh fix' },
  () => {
    const cloud = makePlanarCloud(2, [10, 10, 10, 10]);
    const angles = [359.7, 359.9, 0.1, 0.3];
    for (let ring = 0; ring < 2; ring += 1) {
      for (let column = 0; column < angles.length; column += 1) {
        cloud.azimuths[ring * angles.length + column] =
          (angles[column] * Math.PI) / 180;
      }
    }
    const result = build(cloud);
    assert.equal(result.acceptedQuadCount, 3);
    assert.equal(result.triangleCount, 6);
    assert.equal(result.rejectCounts.azimuth_gap, 0);
    assert.equal(result.rejectCounts.isolated_run, 0);
  }
);
