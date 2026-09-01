export interface EquipmentMetricState {
  seen: boolean;
  source: string;
  quality: string;
  invalid_reason?: string;
  overall_status?: string;
  stamp_ns: number;
  attitude: {
    valid: boolean;
    roll_valid?: boolean;
    pitch_valid?: boolean;
    yaw_valid?: boolean;
    roll_quality?: string;
    pitch_quality?: string;
    yaw_quality?: string;
    roll_invalid_reason?: string;
    pitch_invalid_reason?: string;
    yaw_invalid_reason?: string;
    roll_deg: number;
    pitch_deg: number;
    yaw_deg: number;
    ground_plane_rmse?: number;
    ground_points: number;
    wall_points: number;
    point_count?: number;
  };
  distances: {
    valid: boolean;
    left_front_valid?: boolean;
    left_rear_valid?: boolean;
    right_front_valid?: boolean;
    right_rear_valid?: boolean;
    left_front_quality?: string;
    left_rear_quality?: string;
    right_front_quality?: string;
    right_rear_quality?: string;
    left_front_invalid_reason?: string;
    left_rear_invalid_reason?: string;
    right_front_invalid_reason?: string;
    right_rear_invalid_reason?: string;
    left_front_clearance_m?: number;
    left_rear_clearance_m?: number;
    right_front_clearance_m?: number;
    right_rear_clearance_m?: number;
    left_front_mm: number;
    left_rear_mm: number;
    right_front_mm: number;
    right_rear_mm: number;
    left_front_points: number;
    left_rear_points: number;
    right_front_points: number;
    right_rear_points: number;
  };
}

export interface ViewerStatus {
  connected: boolean;
  cloud_clients: number;
  status_clients: number;
  mesh_clients?: number;
  local_tsdf_mesh_revision?: number;
  local_tsdf_mesh_vertices?: number;
  local_tsdf_mesh_triangles?: number;
  local_tsdf_mesh_packet_bytes?: number;
  local_tsdf_mesh_stamp_ns?: string;
  local_tsdf_mesh_frame_id?: string;
  local_tsdf_mesh_source_topic?: string;
  local_tsdf_mesh_publisher_count?: number;
  local_tsdf_mesh_message_age_ms?: number;
  local_tsdf_mesh_stale_timeout_sec?: number;
  current_cloud_points: number;
  current_cloud_raw_points: number;
  current_cloud_stamp_ns?: string;
  current_cloud_publisher_count?: number;
  current_cloud_frame_id?: string;
  current_cloud_send_rate_hz?: number;
  current_cloud_voxel_size_m?: number;
  current_cloud_max_points?: number;
  current_cloud_transform_body_to_map?: boolean;
  current_cloud_at_point_limit?: boolean;
  stable_map_points: number;
  stable_map_raw_points: number;
  stable_map_frame_id?: string;
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
  cloud_estimate?: EquipmentMetricState;
  real_sensors?: EquipmentMetricState;
  target_xy?: {
    seen: boolean;
    stamp_ns: number;
    center_x_mm: number;
    center_y_mm: number;
    dx_mm: number;
    dy_mm: number;
    velocity_x_mm_s: number;
    velocity_y_mm_s: number;
    status: number;
    status_text: string;
  };
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
