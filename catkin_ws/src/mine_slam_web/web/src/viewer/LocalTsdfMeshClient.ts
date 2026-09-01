const MESH_MAGIC = 0x4d4d5348;
const MESH_VERSION = 1;
const MESH_HEADER_BYTES = 64;
const OP_REPLACE_ALL = 1;
const OP_CLEAR = 2;
const FLAG_HAS_RGBA = 1 << 0;
const FLAG_HAS_UINT32_INDICES = 1 << 1;
const MAX_PACKET_BYTES = 64 * 1024 * 1024;
const MAX_VERTEX_COUNT = 1_000_000;
const MAX_INDEX_COUNT = 3_000_000;

export type LocalTsdfMeshOperation = 'replace' | 'clear';

export interface ParsedLocalTsdfMesh {
  operation: LocalTsdfMeshOperation;
  revision: bigint;
  stampNs: bigint;
  frameId: string;
  vertexCount: number;
  triangleCount: number;
  packetBytes: number;
  positions: Float32Array;
  colors: Uint8Array;
  indices: Uint32Array | null;
}

export type LocalTsdfMeshHandler = (mesh: ParsedLocalTsdfMesh) => void;
export type LocalTsdfMeshConnectionHandler = (connected: boolean) => void;
export type LocalTsdfMeshErrorHandler = (reason: string) => void;

export class LocalTsdfMeshClient {
  private socket: WebSocket | null = null;
  private reconnectTimer = 0;
  private applyFrame = 0;
  private stopped = true;
  private connected = false;
  private socketGeneration = 0;
  private pendingMesh: ParsedLocalTsdfMesh | null = null;
  private lastRevision = 0n;

  constructor(
    private readonly url: string,
    private readonly onMesh: LocalTsdfMeshHandler,
    private readonly onConnection: LocalTsdfMeshConnectionHandler,
    private readonly onError: LocalTsdfMeshErrorHandler = () => undefined
  ) {}

  start(): void {
    if (!this.stopped) return;
    this.stopped = false;
    this.connect();
  }

  stop(): void {
    if (this.stopped) return;
    this.stopped = true;
    this.socketGeneration += 1;
    window.clearTimeout(this.reconnectTimer);
    window.cancelAnimationFrame(this.applyFrame);
    this.applyFrame = 0;
    this.pendingMesh = null;
    const socket = this.socket;
    this.socket = null;
    socket?.close();
    this.setConnected(false);
  }

  private connect(): void {
    if (this.stopped) return;
    const generation = this.socketGeneration + 1;
    this.socketGeneration = generation;
    const socket = new WebSocket(this.url);
    this.socket = socket;
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => {
      if (!this.isCurrentSocket(socket, generation)) return;
      this.lastRevision = 0n;
      this.setConnected(true);
    };
    socket.onclose = () => this.scheduleReconnect(socket, generation);
    socket.onerror = () => this.scheduleReconnect(socket, generation);
    socket.onmessage = (event) => {
      if (!this.isCurrentSocket(socket, generation)) return;
      if (!(event.data instanceof ArrayBuffer)) return;
      const parsed = parseLocalTsdfMeshPacket(event.data);
      if (!parsed) {
        this.onError('invalid_mesh_packet');
        return;
      }
      if (parsed.revision <= this.lastRevision) return;
      this.lastRevision = parsed.revision;
      this.pendingMesh = parsed;
      if (this.applyFrame === 0) {
        this.applyFrame = window.requestAnimationFrame(() => this.flushPending());
      }
    };
  }

  private flushPending(): void {
    this.applyFrame = 0;
    const mesh = this.pendingMesh;
    this.pendingMesh = null;
    if (mesh) this.onMesh(mesh);
  }

  private scheduleReconnect(socket: WebSocket, generation: number): void {
    if (!this.isCurrentSocket(socket, generation)) return;
    this.setConnected(false);
    this.pendingMesh = null;
    window.cancelAnimationFrame(this.applyFrame);
    this.applyFrame = 0;
    if (this.stopped) return;
    window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = window.setTimeout(() => {
      if (this.stopped || this.socketGeneration !== generation) return;
      this.connect();
    }, 1000);
  }

  private isCurrentSocket(socket: WebSocket, generation: number): boolean {
    return (
      !this.stopped &&
      this.socket === socket &&
      this.socketGeneration === generation
    );
  }

  private setConnected(connected: boolean): void {
    if (this.connected === connected) return;
    this.connected = connected;
    this.onConnection(connected);
  }
}

export function parseLocalTsdfMeshPacket(
  buffer: ArrayBuffer
): ParsedLocalTsdfMesh | null {
  if (
    buffer.byteLength < MESH_HEADER_BYTES ||
    buffer.byteLength > MAX_PACKET_BYTES
  ) {
    return null;
  }
  const view = new DataView(buffer);
  if (
    view.getUint32(0, true) !== MESH_MAGIC ||
    view.getUint16(4, true) !== MESH_VERSION
  ) {
    return null;
  }

  const operationValue = view.getUint8(6);
  const flags = view.getUint8(7);
  const revision = view.getBigUint64(8, true);
  const stampNs = view.getBigUint64(16, true);
  const vertexCount = view.getUint32(24, true);
  const indexCount = view.getUint32(28, true);
  const frameIdOffset = view.getUint32(32, true);
  const frameIdBytes = view.getUint32(36, true);
  const positionsOffset = view.getUint32(40, true);
  const colorsOffset = view.getUint32(44, true);
  const indicesOffset = view.getUint32(48, true);
  const packetBytes = view.getUint32(52, true);
  const frameEnd = frameIdOffset + frameIdBytes;
  const alignedFrameEnd = (frameEnd + 3) & ~3;
  if (!Number.isSafeInteger(frameEnd) || alignedFrameEnd < frameEnd) return null;

  if (
    packetBytes !== buffer.byteLength ||
    vertexCount > MAX_VERTEX_COUNT ||
    indexCount > MAX_INDEX_COUNT ||
    frameIdOffset < MESH_HEADER_BYTES ||
    !rangeWithin(frameIdOffset, frameIdBytes, packetBytes)
  ) {
    return null;
  }

  let frameId = '';
  try {
    frameId = new TextDecoder('utf-8', { fatal: true }).decode(
      new Uint8Array(buffer, frameIdOffset, frameIdBytes)
    );
  } catch {
    return null;
  }

  if (operationValue === OP_CLEAR) {
    if (
      vertexCount !== 0 ||
      indexCount !== 0 ||
      flags !== 0 ||
      packetBytes !== alignedFrameEnd ||
      positionsOffset !== packetBytes ||
      colorsOffset !== packetBytes ||
      indicesOffset !== packetBytes
    ) {
      return null;
    }
    return {
      operation: 'clear',
      revision,
      stampNs,
      frameId,
      vertexCount: 0,
      triangleCount: 0,
      packetBytes,
      positions: new Float32Array(0),
      colors: new Uint8Array(0),
      indices: null
    };
  }
  if (operationValue !== OP_REPLACE_ALL || vertexCount === 0) return null;
  if ((flags & FLAG_HAS_RGBA) === 0 || (flags & ~3) !== 0) return null;

  const positionBytes = safeMultiply(vertexCount, 12);
  const colorBytes = safeMultiply(vertexCount, 4);
  if (
    positionBytes === null ||
    colorBytes === null ||
    positionsOffset % 4 !== 0 ||
    positionsOffset !== alignedFrameEnd ||
    !rangeWithin(positionsOffset, positionBytes, packetBytes) ||
    !rangeWithin(colorsOffset, colorBytes, packetBytes) ||
    colorsOffset !== positionsOffset + positionBytes
  ) {
    return null;
  }

  const hasIndices = (flags & FLAG_HAS_UINT32_INDICES) !== 0;
  if ((!hasIndices && indexCount !== 0) || (hasIndices && indexCount === 0)) {
    return null;
  }
  if (
    !hasIndices &&
    (colorsOffset + colorBytes !== packetBytes ||
      indicesOffset !== packetBytes)
  ) {
    return null;
  }
  if (!hasIndices && vertexCount % 3 !== 0) return null;
  const indexBytes = safeMultiply(indexCount, 4);
  if (
    indexBytes === null ||
    (hasIndices &&
      (indicesOffset % 4 !== 0 ||
        indexCount % 3 !== 0 ||
        indicesOffset !== colorsOffset + colorBytes ||
        !rangeWithin(indicesOffset, indexBytes, packetBytes) ||
        indicesOffset + indexBytes !== packetBytes))
  ) {
    return null;
  }

  const positions = new Float32Array(buffer, positionsOffset, vertexCount * 3);
  for (const coordinate of positions) {
    if (!Number.isFinite(coordinate)) return null;
  }
  const colors = new Uint8Array(buffer, colorsOffset, colorBytes);
  const indices = hasIndices
    ? new Uint32Array(buffer, indicesOffset, indexCount)
    : null;
  if (indices) {
    for (const index of indices) {
      if (index >= vertexCount) return null;
    }
  }

  return {
    operation: 'replace',
    revision,
    stampNs,
    frameId,
    vertexCount,
    triangleCount: hasIndices ? indexCount / 3 : vertexCount / 3,
    packetBytes,
    positions,
    colors,
    indices
  };
}

function safeMultiply(left: number, right: number): number | null {
  const value = left * right;
  return Number.isSafeInteger(value) ? value : null;
}

function rangeWithin(offset: number, length: number, total: number): boolean {
  return (
    Number.isSafeInteger(offset) &&
    Number.isSafeInteger(length) &&
    offset >= 0 &&
    length >= 0 &&
    offset <= total &&
    length <= total - offset
  );
}
