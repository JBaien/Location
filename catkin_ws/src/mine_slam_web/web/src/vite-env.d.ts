/// <reference types="vite/client" />

declare module '*.vue' {
  import type { DefineComponent } from 'vue';
  const component: DefineComponent<Record<string, unknown>, Record<string, unknown>, unknown>;
  export default component;
}

interface Window {
  __MINE_SLAM_VIEWER_STATS__?: {
    fps: number;
    frameMs: number;
    frameMeanMs: number;
    frameP95Ms: number;
    currentPoints: number;
    currentRawPoints: number;
    currentCloudStampNs: string;
    currentCloudPublisherCount: number;
    currentCloudAtPointLimit: boolean;
    stablePoints: number;
    pathPoints: number;
    meshTriangles: number;
    meshPointCoverage: number;
    meshQuadCoverage: number;
    meshBuildMs: number;
    meshApplyMs: number;
    meshTopRejectReason: string;
    tsdfMeshConnected: boolean;
    tsdfMeshTriangles: number;
    tsdfMeshRevision: string;
    tsdfMeshPacketErrors: number;
    cloudConnected: boolean;
    statusConnected: boolean;
  };
  __MINE_SLAM_VIEWER_DEBUG__?: {
    meshDebug: () => import('./viewer/ScanMeshLayer').ScanMeshDebug | null;
    meshLatest: () => import('./viewer/ScanMeshLayer').MeshFrameMetrics | null;
    meshFrames: () => import('./viewer/ScanMeshLayer').MeshFrameMetrics[];
    tsdfMesh: () => import('./viewer/LocalTsdfMeshLayer').LocalTsdfMeshDebug | null;
    bridgeFrames: () => Array<{
      stamp_ns: string;
      captured_at_ms: number;
      raw_points: number;
      encoded_points: number;
      encoded_retention: number;
      publisher_count: number | null;
      voxel_size_m: number | null;
      max_points: number | null;
      at_point_limit: boolean;
    }>;
    clearMeshFrames: () => void;
    clearBridgeFrames: () => void;
  };
}
