# Sensors

Each agent template declares its sensors in SDF (see `simapi/engine/gazebo/sdf.py`).
Observations reach external clients only through the Simulation API; nobody
touches Gazebo topics directly.

| Sensor | Gazebo system plugin (world) | Gazebo topic | API surface |
|---|---|---|---|
| IMU | `gz::sim::systems::Imu` | `/<agent>/imu` (`gz.msgs.IMU`) | `observation.imu` |
| NavSat (GPS-like) | `gz::sim::systems::NavSat` + world `<spherical_coordinates>` | `/<agent>/navsat` | planned: `observation.navsat` |
| Odometry (pose/velocity) | `gz::sim::systems::OdometryPublisher` (model) | `/model/<agent>/odometry` | `observation.state` |
| RGB camera | `gz::sim::systems::Sensors` (ogre2) | `/<agent>/camera` (`gz.msgs.Image`) | `GET /agents/{id}/sensors/camera` |
| Depth camera | `gz::sim::systems::Sensors` (ogre2) | `/<agent>/depth` (R_FLOAT32) | `GET /agents/{id}/sensors/depth` |

## Rendering sensors on macOS — verified working

Camera/depth need the Sensors system (`ogre2`). **Tested on macOS 26 / Apple
Silicon with Gazebo Jetty:** `gz sim -s --headless-rendering` renders camera
and depth sensors correctly (Metal backend; ~0.15 s init). Without the flag it
also works but rendering init takes ~6 s. Cameras are attached when a scenario
agent has a `camera:` block; the world then loads the Sensors system.

Observed throughput on an M-series laptop: ~6–7 fps per 320×240 camera at a
configured 15 Hz while physics runs at real time. Reset with cameras attached
costs ~3 s (rendering re-initialises). Physics, IMU, NavSat, contacts and
odometry never need rendering.

| Sensor | Gazebo system plugin (world) | Gazebo topic | API surface |
|---|---|---|---|
| Contact | `gz::sim::systems::Contact` | `/<agent>/contacts` | `collision` / `landing` / `takeoff` events, `observation.grounded` |
| NavSat | `gz::sim::systems::NavSat` | `/<agent>/navsat` | `observation.gps` (profiles with `gps`) |
| RGB / depth | `gz::sim::systems::Sensors` | `/<agent>/camera`, `/<agent>/depth` | `observation.frames[]` refs; `GET /agents/{id}/sensors/{name}`; `WS /ws/sensors/{id}/{name}` |
