import { randomBytes } from 'node:crypto';
import { createConnection, type Socket } from 'node:net';

import { parseCloudPacket } from '../src/viewer/BinaryCloudClient';
import {
  buildStableScanMesh,
  QUAD_REJECT_REASONS
} from '../src/viewer/StableScanMeshBuilder';

const targetFrames = Math.max(1, Number.parseInt(process.argv[2] ?? '100', 10));
const host = process.argv[3] ?? '127.0.0.1';
const port = Math.max(1, Number.parseInt(process.argv[4] ?? '9001', 10));
const timeoutMs = Math.max(10_000, targetFrames * 500);

interface FrameSample {
  stampNs: bigint;
  protocolVersion: number;
  pointCount: number;
  triangleCount: number;
  buildMs: number;
  matchCoverage: number;
  pointCoverage: number;
  quadCoverage: number;
  rangeValidity: number;
  rejectedQuadCount: number;
  rejectCounts: Record<string, number>;
}

let socket: Socket | null = null;
let receiveBuffer = Buffer.alloc(0);
let handshakeComplete = false;
let fragmentedPayloads: Buffer[] = [];
const samples: FrameSample[] = [];
const startedAtMs = performance.now();

const timeout = setTimeout(() => {
  fail(`timed out after ${timeoutMs} ms with ${samples.length}/${targetFrames} frames`);
}, timeoutMs);

function fail(message: string): never {
  clearTimeout(timeout);
  socket?.destroy();
  process.stderr.write(`${message}\n`);
  process.exit(1);
}

function percentile(values: number[], quantile: number): number {
  if (values.length === 0) return 0;
  const sorted = [...values].sort((left, right) => left - right);
  const index = Math.min(
    sorted.length - 1,
    Math.max(0, Math.floor((sorted.length - 1) * quantile))
  );
  return sorted[index];
}

function mean(values: number[]): number {
  if (values.length === 0) return 0;
  return values.reduce((total, value) => total + value, 0) / values.length;
}

function finish(): never {
  clearTimeout(timeout);
  socket?.end();
  const buildTimes = samples.map((sample) => sample.buildMs);
  const triangleCounts = samples.map((sample) => sample.triangleCount);
  const triangleMean = mean(triangleCounts);
  const triangleStdDev = Math.sqrt(
    mean(triangleCounts.map((value) => (value - triangleMean) ** 2))
  );
  const rejectTotals = Object.fromEntries(
    QUAD_REJECT_REASONS.map((reason) => [
      reason,
      samples.reduce((total, sample) => total + sample.rejectCounts[reason], 0)
    ])
  );
  const topRejects = Object.entries(rejectTotals)
    .filter(([, count]) => count > 0)
    .sort((left, right) => right[1] - left[1])
    .slice(0, 8)
    .map(([reason, count]) => ({ reason, count }));
  let stampRegressions = 0;
  for (let index = 1; index < samples.length; index += 1) {
    if (samples[index].stampNs <= samples[index - 1].stampNs) stampRegressions += 1;
  }

  const summary = {
    frames: samples.length,
    elapsed_ms: performance.now() - startedAtMs,
    protocol_versions: [...new Set(samples.map((sample) => sample.protocolVersion))],
    stamp_regressions: stampRegressions,
    points: {
      min: Math.min(...samples.map((sample) => sample.pointCount)),
      mean: mean(samples.map((sample) => sample.pointCount)),
      max: Math.max(...samples.map((sample) => sample.pointCount))
    },
    triangles: {
      min: Math.min(...triangleCounts),
      mean: triangleMean,
      max: Math.max(...triangleCounts),
      coefficient_of_variation: triangleMean > 0 ? triangleStdDev / triangleMean : 0
    },
    coverage: {
      match_p10: percentile(samples.map((sample) => sample.matchCoverage), 0.10),
      match_p50: percentile(samples.map((sample) => sample.matchCoverage), 0.50),
      point_p10: percentile(samples.map((sample) => sample.pointCoverage), 0.10),
      point_p50: percentile(samples.map((sample) => sample.pointCoverage), 0.50),
      quad_p10: percentile(samples.map((sample) => sample.quadCoverage), 0.10),
      quad_p50: percentile(samples.map((sample) => sample.quadCoverage), 0.50),
      range_validity_min: Math.min(...samples.map((sample) => sample.rangeValidity))
    },
    mesh_build_ms: {
      mean: mean(buildTimes),
      p50: percentile(buildTimes, 0.50),
      p95: percentile(buildTimes, 0.95),
      max: Math.max(...buildTimes)
    },
    rejected_quads: samples.reduce(
      (total, sample) => total + sample.rejectedQuadCount,
      0
    ),
    top_rejects: topRejects
  };
  process.stdout.write(`${JSON.stringify(summary, null, 2)}\n`);
  process.exit(0);
}

function consumeCloudPacket(payload: Buffer): void {
  const arrayBuffer = payload.buffer.slice(
    payload.byteOffset,
    payload.byteOffset + payload.byteLength
  );
  const cloud = parseCloudPacket(arrayBuffer);
  if (!cloud || cloud.cloudType !== 1) return;
  const buildStartedAtMs = performance.now();
  const result = buildStableScanMesh(cloud);
  const buildMs = performance.now() - buildStartedAtMs;
  const rejectSum = QUAD_REJECT_REASONS.reduce(
    (total, reason) => total + result.rejectCounts[reason],
    0
  );
  if (
    result.candidateQuadCount !== result.acceptedQuadCount + result.rejectedQuadCount ||
    result.triangleCount !== result.acceptedQuadCount * 2 ||
    rejectSum !== result.rejectedQuadCount
  ) {
    fail(`mesh accounting invariant failed at stamp ${cloud.stampNs}`);
  }
  samples.push({
    stampNs: cloud.stampNs,
    protocolVersion: cloud.protocolVersion,
    pointCount: cloud.pointCount,
    triangleCount: result.triangleCount,
    buildMs,
    matchCoverage: result.matchCoverage,
    pointCoverage: result.pointCoverage,
    quadCoverage: result.quadCoverage,
    rangeValidity: result.rangeValidity,
    rejectedQuadCount: result.rejectedQuadCount,
    rejectCounts: result.rejectCounts
  });
  if (samples.length >= targetFrames) finish();
}

function consumeFrames(): void {
  while (receiveBuffer.length >= 2) {
    const first = receiveBuffer[0];
    const second = receiveBuffer[1];
    const fin = (first & 0x80) !== 0;
    const opcode = first & 0x0f;
    const masked = (second & 0x80) !== 0;
    let payloadLength = second & 0x7f;
    let headerLength = 2;
    if (payloadLength === 126) {
      if (receiveBuffer.length < 4) return;
      payloadLength = receiveBuffer.readUInt16BE(2);
      headerLength = 4;
    } else if (payloadLength === 127) {
      if (receiveBuffer.length < 10) return;
      const length = receiveBuffer.readBigUInt64BE(2);
      if (length > BigInt(Number.MAX_SAFE_INTEGER)) fail('WebSocket frame is too large');
      payloadLength = Number(length);
      headerLength = 10;
    }
    const maskLength = masked ? 4 : 0;
    const frameLength = headerLength + maskLength + payloadLength;
    if (receiveBuffer.length < frameLength) return;
    let payload = receiveBuffer.subarray(headerLength + maskLength, frameLength);
    if (masked) {
      const mask = receiveBuffer.subarray(headerLength, headerLength + 4);
      payload = Buffer.from(payload);
      for (let index = 0; index < payload.length; index += 1) {
        payload[index] ^= mask[index % 4];
      }
    }
    receiveBuffer = receiveBuffer.subarray(frameLength);

    if (opcode === 0x8) fail('WebSocket server closed the connection');
    if (opcode === 0x9) continue;
    if (opcode === 0x2 || opcode === 0x0) fragmentedPayloads.push(Buffer.from(payload));
    if (fin && (opcode === 0x2 || opcode === 0x0)) {
      consumeCloudPacket(Buffer.concat(fragmentedPayloads));
      fragmentedPayloads = [];
    }
  }
}

socket = createConnection({ host, port }, () => {
  const key = randomBytes(16).toString('base64');
  socket?.write(
    `GET /cloud HTTP/1.1\r\n` +
      `Host: ${host}:${port}\r\n` +
      'Upgrade: websocket\r\n' +
      'Connection: Upgrade\r\n' +
      `Sec-WebSocket-Key: ${key}\r\n` +
      'Sec-WebSocket-Version: 13\r\n\r\n'
  );
});

socket.on('data', (chunk) => {
  receiveBuffer = Buffer.concat([receiveBuffer, chunk]);
  if (!handshakeComplete) {
    const boundary = receiveBuffer.indexOf('\r\n\r\n');
    if (boundary < 0) return;
    const response = receiveBuffer.subarray(0, boundary).toString('utf8');
    if (!response.startsWith('HTTP/1.1 101')) fail(`WebSocket handshake failed: ${response}`);
    receiveBuffer = receiveBuffer.subarray(boundary + 4);
    handshakeComplete = true;
  }
  consumeFrames();
});
socket.on('error', (error) => fail(error.message));
socket.on('close', () => {
  if (samples.length < targetFrames) fail(`connection closed at ${samples.length} frames`);
});
