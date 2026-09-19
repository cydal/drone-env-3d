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

## Rendering sensors on macOS

Camera/depth need the Sensors system with a render engine. Gazebo's
`--headless-rendering` is EGL-based and Linux-only; on macOS the server can
still render via Metal when a display session exists, but this is untested
here. The vertical slice therefore enables cameras only when a scenario agent
sets `sensors: {camera: {...}}`; the world then loads the Sensors system.
Physics, IMU, NavSat and odometry never need rendering and work fully headless.

For batch/remote runs with cameras, plan on Linux (native or a `ros:lyrical`
arm64 container with `ros-lyrical-ros-gz`).
