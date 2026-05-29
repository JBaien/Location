export type ColorMode =
  | 'height'
  | 'intensity'
  | 'x-distance'
  | 'y-distance'
  | 'z-distance'
  | 'lidar_id'
  | 'stable-temporary'
  | 'reflector';

export interface ColorPoint {
  x: number;
  y: number;
  z: number;
  intensity: number;
  lidarId: number;
  classId: number;
  cloudType: number;
}

export function colorForPoint(point: ColorPoint, mode: ColorMode, reflectorVisible: boolean): [number, number, number] {
  if (reflectorVisible && (point.classId === 3 || (mode === 'reflector' && point.intensity >= 180))) {
    return mode === 'reflector' && point.classId === 3 ? [0.1, 1.0, 0.35] : [1.0, 0.86, 0.1];
  }

  if (mode === 'height') {
    const t = clamp((point.z + 2.0) / 5.0);
    if (t < 0.5) {
      const k = t * 2.0;
      return [0.05, 0.2 + 0.65 * k, 1.0 - 0.75 * k];
    }
    const k = (t - 0.5) * 2.0;
    return [0.1 + 0.9 * k, 0.85 - 0.55 * k, 0.1];
  }

  if (mode === 'intensity') {
    const t = clamp(point.intensity / 255.0);
    return [0.12 + 0.88 * t, 0.12 + 0.78 * t, 0.12 + 0.2 * t];
  }

  if (mode === 'x-distance') {
    return distanceRamp(Math.abs(point.x) / 100.0);
  }

  if (mode === 'y-distance') {
    return distanceRamp(Math.abs(point.y) / 8.0);
  }

  if (mode === 'z-distance') {
    return distanceRamp(Math.abs(point.z) / 5.0);
  }

  if (mode === 'lidar_id') {
    if (point.lidarId === 0) return [0.15, 0.45, 1.0];
    if (point.lidarId === 1) return [0.15, 0.95, 0.45];
    if (point.lidarId === 2) return [0.75, 0.35, 1.0];
    return [0.55, 0.55, 0.55];
  }

  if (mode === 'stable-temporary') {
    return point.cloudType === 2 ? [0.2, 0.62, 1.0] : [0.78, 0.8, 0.76];
  }

  return [0.55, 0.55, 0.55];
}

function clamp(value: number): number {
  return Math.max(0, Math.min(1, value));
}

function distanceRamp(value: number): [number, number, number] {
  const t = clamp(value);
  if (t < 0.33) {
    const k = t / 0.33;
    return [0.05, 0.25 + 0.65 * k, 1.0 - 0.55 * k];
  }
  if (t < 0.66) {
    const k = (t - 0.33) / 0.33;
    return [0.05 + 0.85 * k, 0.9, 0.45 - 0.35 * k];
  }
  const k = (t - 0.66) / 0.34;
  return [0.9 + 0.1 * k, 0.9 - 0.75 * k, 0.1];
}
