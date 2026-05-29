export interface ViewerStatus {
  connected: boolean;
  cloud_clients: number;
  status_clients: number;
  current_cloud_points: number;
  current_cloud_raw_points: number;
  stable_map_points: number;
  stable_map_raw_points: number;
  latest_current_cloud_cached?: boolean;
  latest_stable_map_cached?: boolean;
  latest_path_cached?: boolean;
  initial_snapshot_reason?: string;
  session_id?: number;
  current_cloud_source_topic?: string;
  stable_map_source_topic?: string;
  current_layer_semantics?: string;
  stable_layer_semantics?: string;
  pose: { valid: boolean; x: number; y: number; z: number; qx: number; qy: number; qz: number; qw: number };
  path: number[][];
  map_frame: string;
  path_frame?: string;
  path_source?: string;
  full_path_point_count?: number;
  odom_path_point_count?: number;
  displayed_path_point_count?: number;
  status_path_point_count?: number;
  status_path_stride?: number;
  max_path_points?: number;
  path_is_snapshot_or_delta?: string;
  path_start_stamp_ns?: number;
  path_end_stamp_ns?: number;
  path_start_s?: number;
  path_end_s?: number;
  path_snapshot_span_s?: number;
  path_odom_span_s?: number;
  path_snapshot_complete?: boolean;
  odom_source?: string;
  path_reset_count?: number;
  last_path_reset_reason?: string;
  fixed_frame?: string;
  pointcloud_frame?: string;
  pose_frame?: string;
  transform_applied_in_backend?: boolean;
  transform_applied_in_frontend?: boolean;
  transform_axis_mapping?: string;
  double_transform_detected?: boolean;
  progressive_reveal_seen?: boolean;
  progressive_reveal_enabled?: boolean;
  progressive_machine_s?: number;
  progressive_revealed_s?: number;
  progressive_face_wall_s?: number;
  progressive_visible_reflector_count?: number;
  progressive_hidden_reflector_count?: number;
  progressive_hidden_unrevealed_point_count?: number;
  progressive_filter_front_unrevealed_point_count?: number;
  progressive_published_face_point_count?: number;
  progressive_reveal_source?: string;
}

export class StatusClient {
  private socket: WebSocket | null = null;
  private reconnectTimer = 0;
  private stopped = false;

  constructor(
    private readonly url: string,
    private readonly onStatus: (status: ViewerStatus) => void,
    private readonly onConnection: (connected: boolean) => void
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
    this.socket.onopen = () => this.onConnection(true);
    this.socket.onclose = () => this.scheduleReconnect();
    this.socket.onerror = () => this.scheduleReconnect();
    this.socket.onmessage = (event) => {
      if (typeof event.data !== 'string') return;
      const status = JSON.parse(event.data) as ViewerStatus;
      this.onStatus(status);
    };
  }

  private scheduleReconnect(): void {
    this.onConnection(false);
    if (this.stopped) return;
    window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = window.setTimeout(() => this.connect(), 1000);
  }
}
