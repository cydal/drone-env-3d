"""SDF generation: world template + entity templates.

Physical parameters of the quadcopter template are taken from the X3 UAV model
that ships with gz-sim's multicopter_velocity_control example world (1.5 kg
airframe, 4 rotors) so the MulticopterVelocityControl gains are known-stable.
Visuals are primitives so the browser needs no mesh loading.
"""
from __future__ import annotations

import math
from xml.sax.saxutils import escape

from ...models import Pose
from ...scenario import AgentSpec, CameraSpec, EntitySpec, Scenario


def pose_str(p: Pose) -> str:
    x, y, z, w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return f"{p.position.x} {p.position.y} {p.position.z} {roll} {pitch} {yaw}"


def _material(rgba: str) -> str:
    return (f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
            f"<specular>0.2 0.2 0.2 1</specular></material>")


# ---------------------------------------------------------------------------
# Quadcopter agent template
# ---------------------------------------------------------------------------

_ROTORS = [
    # name, (x, y, z), turning direction, controller direction sign
    ("rotor_0", (0.13, -0.22, 0.023), "ccw", 1),
    ("rotor_1", (-0.13, 0.20, 0.023), "ccw", 1),
    ("rotor_2", (0.13, 0.22, 0.023), "cw", -1),
    ("rotor_3", (-0.13, -0.20, 0.023), "cw", -1),
]
MOTOR_CONSTANT = 8.54858e-06
MOMENT_CONSTANT = 0.016


def _camera_sensors(ns: str, cam: CameraSpec) -> str:
    pose = " ".join(str(v) for v in cam.pose)
    out = f"""
      <sensor name="camera" type="camera">
        <pose>{pose}</pose>
        <topic>/{ns}/camera</topic>
        <update_rate>{cam.update_rate}</update_rate>
        <camera>
          <horizontal_fov>{cam.hfov}</horizontal_fov>
          <image><width>{cam.width}</width><height>{cam.height}</height><format>RGB_INT8</format></image>
          <clip><near>0.05</near><far>300</far></clip>
        </camera>
        <always_on>1</always_on>
      </sensor>"""
    if cam.depth:
        out += f"""
      <sensor name="depth" type="depth_camera">
        <pose>{pose}</pose>
        <topic>/{ns}/depth</topic>
        <update_rate>{cam.update_rate}</update_rate>
        <camera>
          <horizontal_fov>{cam.hfov}</horizontal_fov>
          <image><width>{cam.width}</width><height>{cam.height}</height><format>R_FLOAT32</format></image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
        <always_on>1</always_on>
      </sensor>"""
    return out


def quadcopter_sdf(entity_id: str, params: dict | None = None, *, camera: CameraSpec | None = None,
                   wrap_in_sdf: bool = True) -> str:
    """Return the <model> (optionally wrapped in <sdf>) for a quadcopter agent."""
    params = params or {}
    ns = entity_id
    color = params.get("color", "0.95 0.45 0.10 1")

    rotors_links, rotors_joints, motor_plugins, rotor_cfg = [], [], [], []
    for name, (x, y, z), turning, direction in _ROTORS:
        rotors_links.append(f"""
    <link name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>9.75e-07</ixx><iyy>4.17041e-05</iyy><izz>4.26041e-05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="{name}_collision">
        <geometry><cylinder><length>0.005</length><radius>0.1</radius></cylinder></geometry>
        <surface><contact><ode/></contact><friction><ode/></friction></surface>
      </collision>
      <visual name="{name}_visual">
        <geometry><cylinder><length>0.006</length><radius>0.1</radius></cylinder></geometry>
        {_material("0.15 0.15 0.15 0.85")}
      </visual>
    </link>""")
        rotors_joints.append(f"""
    <joint name="{name}_joint" type="revolute">
      <child>{name}</child>
      <parent>base_link</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
      </axis>
    </joint>""")
        motor_plugins.append(f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>{ns}</robotNamespace>
      <jointName>{name}_joint</jointName>
      <linkName>{name}</linkName>
      <turningDirection>{turning}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>800.0</maxRotVelocity>
      <motorConstant>{MOTOR_CONSTANT}</motorConstant>
      <momentConstant>{MOMENT_CONSTANT}</momentConstant>
      <commandSubTopic>cmd/motor_speed</commandSubTopic>
      <actuator_number>{name[-1]}</actuator_number>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>""")
        rotor_cfg.append(f"""
        <rotor>
          <jointName>{name}_joint</jointName>
          <forceConstant>{MOTOR_CONSTANT}</forceConstant>
          <momentConstant>{MOMENT_CONSTANT}</momentConstant>
          <direction>{direction}</direction>
        </rotor>""")

    arms = ""
    for i, yaw in enumerate((0.9828, -0.9828)):  # atan2(0.22, 0.13)
        arms += f"""
      <visual name="arm_{i}">
        <pose>0 0 0.01 0 0 {yaw}</pose>
        <geometry><box><size>0.52 0.03 0.015</size></box></geometry>
        {_material("0.25 0.25 0.28 1")}
      </visual>"""

    cameras = _camera_sensors(ns, camera) if camera else ""

    model = f"""
  <model name="{escape(entity_id)}">
    <link name="base_link">
      <inertial>
        <mass>1.5</mass>
        <inertia><ixx>0.0347563</ixx><iyy>0.07</iyy><izz>0.0977</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="base_collision">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry><box><size>0.47 0.47 0.11</size></box></geometry>
      </collision>
      <visual name="body">
        <geometry><box><size>0.18 0.12 0.06</size></box></geometry>
        {_material(color)}
      </visual>
      <visual name="nose">
        <pose>0.11 0 0 0 0 0</pose>
        <geometry><box><size>0.05 0.06 0.03</size></box></geometry>
        {_material("0.1 0.1 0.1 1")}
      </visual>{arms}
      <sensor name="imu" type="imu">
        <topic>/{ns}/imu</topic>
        <update_rate>200</update_rate>
        <always_on>1</always_on>
      </sensor>
      <sensor name="navsat" type="navsat">
        <topic>/{ns}/navsat</topic>
        <update_rate>10</update_rate>
        <always_on>1</always_on>
      </sensor>
      <sensor name="contact" type="contact">
        <contact>
          <collision>base_collision</collision>
          <topic>/{ns}/contacts</topic>
        </contact>
        <update_rate>50</update_rate>
        <always_on>1</always_on>
      </sensor>{cameras}
    </link>
    {''.join(rotors_links)}
    {''.join(rotors_joints)}
    {''.join(motor_plugins)}
    <plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl">
      <robotNamespace>{ns}</robotNamespace>
      <commandSubTopic>cmd/twist</commandSubTopic>
      <enableSubTopic>cmd/enable</enableSubTopic>
      <comLinkName>base_link</comLinkName>
      <velocityGain>2.7 2.7 2.7</velocityGain>
      <attitudeGain>2 3 0.15</attitudeGain>
      <angularRateGain>0.4 0.52 0.18</angularRateGain>
      <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
      <rotorConfiguration>{''.join(rotor_cfg)}
      </rotorConfiguration>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
  </model>"""
    if wrap_in_sdf:
        return f'<?xml version="1.0"?>\n<sdf version="1.9">{model}\n</sdf>\n'
    return model


# ---------------------------------------------------------------------------
# Environment-controlled (kinematic) entity templates
# ---------------------------------------------------------------------------

def target_sdf(entity_id: str, params: dict | None = None, *, wrap_in_sdf: bool = True, **_) -> str:
    """A floating marker sphere. Gravity off; moved kinematically by the environment."""
    params = params or {}
    r = float(params.get("radius", 0.5))
    color = params.get("color", "1.0 0.2 0.2 1")
    collision = "" if params.get("collide", True) in (False, "false", 0) else \
        f'<collision name="collision"><geometry><sphere><radius>{r}</radius></sphere></geometry></collision>'
    model = f"""
  <model name="{escape(entity_id)}">
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      {collision}
      <visual name="visual"><geometry><sphere><radius>{r}</radius></sphere></geometry>{_material(color)}</visual>
      <visual name="ring">
        <geometry><cylinder><radius>{r * 1.6}</radius><length>0.03</length></cylinder></geometry>
        {_material("1 1 1 0.6")}
      </visual>
    </link>
  </model>"""
    return f'<?xml version="1.0"?>\n<sdf version="1.9">{model}\n</sdf>\n' if wrap_in_sdf else model


def vehicle_sdf(entity_id: str, params: dict | None = None, *, wrap_in_sdf: bool = True, **_) -> str:
    """A ground vehicle body (box + wheels). Gravity off; moved kinematically."""
    params = params or {}
    color = params.get("color", "0.2 0.55 0.95 1")
    L, W, H = 4.2, 1.9, 1.3
    wheels = ""
    for i, (x, y) in enumerate(((1.4, 1.0), (1.4, -1.0), (-1.4, 1.0), (-1.4, -1.0))):
        wheels += f"""
      <visual name="wheel_{i}">
        <pose>{x} {y} -0.35 1.5708 0 0</pose>
        <geometry><cylinder><radius>0.38</radius><length>0.3</length></cylinder></geometry>
        {_material("0.08 0.08 0.08 1")}
      </visual>"""
    model = f"""
  <model name="{escape(entity_id)}">
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>1500</mass><inertia><ixx>800</ixx><iyy>2500</iyy><izz>2800</izz></inertia></inertial>
      <collision name="collision"><geometry><box><size>{L} {W} {H}</size></box></geometry></collision>
      <visual name="body"><geometry><box><size>{L} {W} {H * 0.55}</size></box></geometry>{_material(color)}</visual>
      <visual name="cabin">
        <pose>-0.3 0 {H * 0.45} 0 0 0</pose>
        <geometry><box><size>{L * 0.5} {W * 0.9} {H * 0.45}</size></box></geometry>
        {_material("0.15 0.18 0.22 1")}
      </visual>{wheels}
    </link>
  </model>"""
    return f'<?xml version="1.0"?>\n<sdf version="1.9">{model}\n</sdf>\n' if wrap_in_sdf else model


AGENT_TEMPLATES = {"quadcopter": quadcopter_sdf}
ENTITY_TEMPLATES = {"target": target_sdf, "vehicle": vehicle_sdf}
TEMPLATES = {**AGENT_TEMPLATES, **ENTITY_TEMPLATES}


def _with_pose(model: str, pose: Pose) -> str:
    head, sep, tail = model.partition(">")
    return f"{head}{sep}\n    <pose>{pose_str(pose)}</pose>{tail}"


# ---------------------------------------------------------------------------
# World assembly: base world file + scenario -> concrete SDF
# ---------------------------------------------------------------------------

def build_world_sdf(base_world_sdf: str, scenario: Scenario) -> str:
    """Inject physics/system plugins, agents and dynamic entities into a base world.

    The base world must contain the marker comments
    ``<!-- @physics -->`` and ``<!-- @agents -->``.
    """
    sim = scenario.simulation
    physics = f"""
    <physics name="default" type="dart">
      <max_step_size>{sim.step_size}</max_step_size>
      <real_time_factor>{sim.real_time_factor}</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
      <!-- >= physics rate so every iteration publishes: the pose read after a step is exact -->
      <dynamic_pose_hertz>{max(60, int(round(1.0 / sim.step_size)))}</dynamic_pose_hertz>
      <state_hertz>10</state_hertz>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>"""
    if scenario.rendering:
        physics += """
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>"""
    w = scenario.environment.wind
    if w is not None and (w.x or w.y or w.z):
        physics += f"""
    <wind><linear_velocity>{w.x} {w.y} {w.z}</linear_velocity></wind>"""

    models = []
    for a in scenario.agents:
        tpl = AGENT_TEMPLATES[a.template]
        models.append(_with_pose(tpl(a.id, a.params, camera=a.camera, wrap_in_sdf=False), a.spawn))
    for e in scenario.entities:
        tpl = ENTITY_TEMPLATES[e.template]
        models.append(_with_pose(tpl(e.id, e.params, wrap_in_sdf=False), e.spawn))

    out = base_world_sdf.replace("<!-- @physics -->", physics)
    out = out.replace("<!-- @agents -->", "\n".join(models))
    return out
