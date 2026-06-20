# ZF Runtime

This Docker runtime is for the ZF two-Timoo-LiDAR realtime point cloud view.

It uses:

- Timoo TM16 legacy driver family: `driver_family: timoo`
- LiDAR topics: `/lidar1/timoo_points`, `/lidar2/timoo_points`
- Fused realtime cloud: `/points_raw`
- Web current cloud: `/points_raw`
- Web backend map transform: disabled
- Target localizer: disabled by the ZF compose command

Start on x86:

```bash
docker compose -f docker/ZF/docker-compose.yml up -d
```

Start on arm64:

```bash
docker compose -f docker/ZF/docker-compose.arm64.yml up -d
```

Open the web page on port `18080`.
