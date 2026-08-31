import assert from 'node:assert/strict';
import test from 'node:test';

import {
  LocalTsdfMeshClient,
  parseLocalTsdfMeshPacket
} from '../src/viewer/LocalTsdfMeshClient';

const HEADER_BYTES = 64;
const MAGIC = 0x4d4d5348;

function align4(value: number): number {
  return (value + 3) & ~3;
}

function makeReplacePacket(indexed = false): ArrayBuffer {
  const frame = new TextEncoder().encode('velodyne');
  const vertexCount = 3;
  const indexCount = indexed ? 3 : 0;
  const positionsOffset = align4(HEADER_BYTES + frame.length);
  const colorsOffset = positionsOffset + vertexCount * 12;
  const indicesOffset = colorsOffset + vertexCount * 4;
  const packetBytes = indicesOffset + indexCount * 4;
  const buffer = new ArrayBuffer(packetBytes);
  const view = new DataView(buffer);
  view.setUint32(0, MAGIC, true);
  view.setUint16(4, 1, true);
  view.setUint8(6, 1);
  view.setUint8(7, indexed ? 3 : 1);
  view.setBigUint64(8, 4n, true);
  view.setBigUint64(16, 123n, true);
  view.setUint32(24, vertexCount, true);
  view.setUint32(28, indexCount, true);
  view.setUint32(32, HEADER_BYTES, true);
  view.setUint32(36, frame.length, true);
  view.setUint32(40, positionsOffset, true);
  view.setUint32(44, colorsOffset, true);
  view.setUint32(48, indexed ? indicesOffset : packetBytes, true);
  view.setUint32(52, packetBytes, true);
  new Uint8Array(buffer, HEADER_BYTES, frame.length).set(frame);
  new Float32Array(buffer, positionsOffset, 9).set([
    0, 0, 0,
    1, 0, 0,
    0, 1, 0
  ]);
  new Uint8Array(buffer, colorsOffset, 12).fill(255);
  if (indexed) new Uint32Array(buffer, indicesOffset, 3).set([0, 1, 2]);
  return buffer;
}

function makeClearPacket(): ArrayBuffer {
  const frame = new TextEncoder().encode('map');
  const packetBytes = align4(HEADER_BYTES + frame.length);
  const buffer = new ArrayBuffer(packetBytes);
  const view = new DataView(buffer);
  view.setUint32(0, MAGIC, true);
  view.setUint16(4, 1, true);
  view.setUint8(6, 2);
  view.setBigUint64(8, 5n, true);
  view.setUint32(32, HEADER_BYTES, true);
  view.setUint32(36, frame.length, true);
  view.setUint32(40, packetBytes, true);
  view.setUint32(44, packetBytes, true);
  view.setUint32(48, packetBytes, true);
  view.setUint32(52, packetBytes, true);
  new Uint8Array(buffer, HEADER_BYTES, frame.length).set(frame);
  return buffer;
}

test('parses valid non-indexed replace and clear snapshots', () => {
  const replace = parseLocalTsdfMeshPacket(makeReplacePacket());
  assert.ok(replace);
  assert.equal(replace.operation, 'replace');
  assert.equal(replace.revision, 4n);
  assert.equal(replace.frameId, 'velodyne');
  assert.equal(replace.vertexCount, 3);
  assert.equal(replace.triangleCount, 1);
  assert.equal(replace.indices, null);

  const clear = parseLocalTsdfMeshPacket(makeClearPacket());
  assert.ok(clear);
  assert.equal(clear.operation, 'clear');
  assert.equal(clear.vertexCount, 0);
});

test('rejects truncated and overlapping payload sections', () => {
  const truncated = makeReplacePacket();
  new DataView(truncated).setUint32(52, truncated.byteLength + 4, true);
  assert.equal(parseLocalTsdfMeshPacket(truncated), null);

  const overlapping = makeReplacePacket();
  new DataView(overlapping).setUint32(44, HEADER_BYTES, true);
  assert.equal(parseLocalTsdfMeshPacket(overlapping), null);

  const source = makeReplacePacket();
  const trailing = new ArrayBuffer(source.byteLength + 4);
  new Uint8Array(trailing).set(new Uint8Array(source));
  const trailingView = new DataView(trailing);
  trailingView.setUint32(48, trailing.byteLength, true);
  trailingView.setUint32(52, trailing.byteLength, true);
  assert.equal(parseLocalTsdfMeshPacket(trailing), null);
});

test('rejects non-finite positions and out-of-range indices', () => {
  const nonFinite = makeReplacePacket();
  const positionOffset = new DataView(nonFinite).getUint32(40, true);
  new DataView(nonFinite).setFloat32(positionOffset, Number.NaN, true);
  assert.equal(parseLocalTsdfMeshPacket(nonFinite), null);

  const indexed = makeReplacePacket(true);
  const indexOffset = new DataView(indexed).getUint32(48, true);
  new DataView(indexed).setUint32(indexOffset + 8, 3, true);
  assert.equal(parseLocalTsdfMeshPacket(indexed), null);
});

test('ignores stale socket callbacks after reconnect and reports disconnect once', () => {
  const timerCallbacks = new Map<number, () => void>();
  const frameCallbacks = new Map<number, () => void>();
  let nextCallbackId = 1;
  const fakeWindow = {
    setTimeout(callback: () => void): number {
      const id = nextCallbackId++;
      timerCallbacks.set(id, callback);
      return id;
    },
    clearTimeout(id: number): void {
      timerCallbacks.delete(id);
    },
    requestAnimationFrame(callback: FrameRequestCallback): number {
      const id = nextCallbackId++;
      frameCallbacks.set(id, () => callback(0));
      return id;
    },
    cancelAnimationFrame(id: number): void {
      frameCallbacks.delete(id);
    }
  } as unknown as Window & typeof globalThis;

  class FakeWebSocket {
    static instances: FakeWebSocket[] = [];
    binaryType: BinaryType = 'blob';
    onopen: ((event: Event) => void) | null = null;
    onclose: ((event: CloseEvent) => void) | null = null;
    onerror: ((event: Event) => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    closed = false;

    constructor(readonly url: string) {
      FakeWebSocket.instances.push(this);
    }

    close(): void {
      this.closed = true;
    }

    open(): void {
      this.onopen?.({} as Event);
    }

    networkClose(): void {
      this.onclose?.({} as CloseEvent);
    }

    error(): void {
      this.onerror?.({} as Event);
    }

    message(data: ArrayBuffer): void {
      this.onmessage?.({ data } as MessageEvent);
    }
  }

  const originalWindow = Object.getOwnPropertyDescriptor(globalThis, 'window');
  const originalWebSocket = Object.getOwnPropertyDescriptor(
    globalThis,
    'WebSocket'
  );
  Object.defineProperty(globalThis, 'window', {
    configurable: true,
    value: fakeWindow
  });
  Object.defineProperty(globalThis, 'WebSocket', {
    configurable: true,
    value: FakeWebSocket
  });

  const connectionEvents: boolean[] = [];
  const revisions: bigint[] = [];
  const client = new LocalTsdfMeshClient(
    'ws://localhost:9003/mesh',
    (mesh) => revisions.push(mesh.revision),
    (connected) => connectionEvents.push(connected)
  );
  try {
    client.start();
    assert.equal(FakeWebSocket.instances.length, 1);
    const first = FakeWebSocket.instances[0];
    first.open();
    first.message(makeReplacePacket());
    for (const callback of [...frameCallbacks.values()]) callback();
    frameCallbacks.clear();
    assert.deepEqual(revisions, [4n]);

    first.networkClose();
    assert.deepEqual(connectionEvents, [true, false]);
    for (const callback of [...timerCallbacks.values()]) callback();
    timerCallbacks.clear();
    assert.equal(FakeWebSocket.instances.length, 2);

    const second = FakeWebSocket.instances[1];
    second.open();
    assert.deepEqual(connectionEvents, [true, false, true]);

    first.error();
    first.networkClose();
    assert.deepEqual(connectionEvents, [true, false, true]);
    assert.equal(timerCallbacks.size, 0);

    second.message(makeReplacePacket());
    for (const callback of [...frameCallbacks.values()]) callback();
    frameCallbacks.clear();
    assert.deepEqual(revisions, [4n, 4n]);
  } finally {
    client.stop();
    if (originalWindow) {
      Object.defineProperty(globalThis, 'window', originalWindow);
    } else {
      Reflect.deleteProperty(globalThis, 'window');
    }
    if (originalWebSocket) {
      Object.defineProperty(globalThis, 'WebSocket', originalWebSocket);
    } else {
      Reflect.deleteProperty(globalThis, 'WebSocket');
    }
  }
});
