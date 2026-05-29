import { ParsedCloud } from './PointCloudLayer';

const MAGIC = 0x4d504344;
const HEADER_BYTES = 24;
const POINT_BYTES = 21;

export type CloudHandler = (cloud: ParsedCloud) => void;
export type ConnectionHandler = (connected: boolean) => void;

export class BinaryCloudClient {
  private socket: WebSocket | null = null;
  private reconnectTimer = 0;
  private stopped = false;

  constructor(
    private readonly url: string,
    private readonly onCloud: CloudHandler,
    private readonly onConnection: ConnectionHandler
  ) {}

  start(): void {
    this.stopped = false;
    this.connect();
  }

  stop(): void {
    this.stopped = true;
    window.clearTimeout(this.reconnectTimer);
    this.socket?.close();
  }

  private connect(): void {
    this.socket = new WebSocket(this.url);
    this.socket.binaryType = 'arraybuffer';
    this.socket.onopen = () => this.onConnection(true);
    this.socket.onclose = () => this.scheduleReconnect();
    this.socket.onerror = () => this.scheduleReconnect();
    this.socket.onmessage = (event) => {
      if (event.data instanceof ArrayBuffer) {
        const parsed = parseCloudPacket(event.data);
        if (parsed) this.onCloud(parsed);
      }
    };
  }

  private scheduleReconnect(): void {
    this.onConnection(false);
    if (this.stopped) return;
    window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = window.setTimeout(() => this.connect(), 1000);
  }
}

function parseCloudPacket(buffer: ArrayBuffer): ParsedCloud | null {
  if (buffer.byteLength < HEADER_BYTES) return null;
  const view = new DataView(buffer);
  const magic = view.getUint32(0, true);
  const version = view.getUint16(4, true);
  if (magic !== MAGIC || version !== 1) return null;

  const cloudType = view.getUint16(6, true);
  const stampNs = view.getBigUint64(8, true);
  const pointCount = view.getUint32(16, true);
  const fieldsMask = view.getUint32(20, true);
  const expectedBytes = HEADER_BYTES + pointCount * POINT_BYTES;
  if (buffer.byteLength < expectedBytes) return null;

  const positions = new Float32Array(pointCount * 3);
  const intensities = new Float32Array(pointCount);
  const lidarIds = new Uint8Array(pointCount);
  const classIds = new Uint8Array(pointCount);

  let offset = HEADER_BYTES;
  for (let i = 0; i < pointCount; i += 1) {
    positions[i * 3] = view.getFloat32(offset, true);
    positions[i * 3 + 1] = view.getFloat32(offset + 4, true);
    positions[i * 3 + 2] = view.getFloat32(offset + 8, true);
    intensities[i] = view.getFloat32(offset + 12, true);
    lidarIds[i] = view.getUint8(offset + 16);
    classIds[i] = view.getUint8(offset + 17);
    offset += POINT_BYTES;
  }

  return { cloudType, stampNs, pointCount, fieldsMask, positions, intensities, lidarIds, classIds };
}
