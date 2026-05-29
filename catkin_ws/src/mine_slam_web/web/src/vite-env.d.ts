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
    currentPoints: number;
    stablePoints: number;
    pathPoints: number;
    cloudConnected: boolean;
    statusConnected: boolean;
  };
}
