export interface ParsedCloud {
  protocolVersion: number;
  cloudType: number;
  stampNs: bigint;
  pointCount: number;
  fieldsMask: number;
  positions: Float32Array;
  intensities: Float32Array;
  times: Float32Array;
  rings: Uint16Array;
  lidarIds: Uint8Array;
  classIds: Uint8Array;
  hasLidarId: boolean;
  hasRing: boolean;
  hasTime: boolean;
}
