# Pending improvements to the MIRA ↔ ROS 2 integration

Work that is still open after phases 1, 2, 3 and 3 bis of the MIRA ↔ ROS 2 analysis. Phases 1 to 3 bis (RPC timeouts, lifecycle, per-module property keys, startup order, IMU, remaining 2.6 fixes, MIRA arguments from ROS, `MiraModule` base class, `ChannelBridge`/`Inspector`, namespaced authorities, asynchronous `setVelocity`, full `DriveStatus`, `cmd_vel_applied`) are implemented and tested on the Morphia.

Items marked **[Morphia]** apply to the Morphia hardware (`MotorControllerHG4`, a single bumper, a single safety laser, no IR sensors, no magnetic barrier). Items marked **[other robots]** only make sense for other Tory/SCITOS variants.

## Phase 4: robot model and control

### 4.1 Complete `DriveStatus`

The analysis still lists it as pending, but the detailed status table marks 2.3 (the same work) as done and verified on the robot. Most likely this is only an inconsistency in the document, to be confirmed and closed.

### 4.3 IR sensors as `sensor_msgs/Range` and speed limiter **[other robots]**

- Publish `SpeedLimiterActive`/`SpeedLimiterValue` and the IR readings as `sensor_msgs/Range` (the `ir/IRSensor0xFrame` frames already exist in MIRA).
- With `SpeedLimiterMode=1` and `SpeedLimit=0.05` the robot limits its speed to 5 cm/s when an IR sensor detects something closer than 0.54 m, and Nav2 does not know about it.
- Does not apply to the Morphia: it has no IR sensors and the `IRSensor00` blocks were removed from `Tory.xml`.

### 4.4 Stop reasons

- Expose `StopReason`, `SlowReason`, `WaitReason`, `SafetyStatus` and `FPGASafetyState` as diagnostics.
- They do not appear as channels on the Morphia. Check whether they are properties and, if so, read them through `ChannelBridge` or `Inspector`.

### 4.5 Per-module diagnostics, remaining part

`Drive`, `Charger`, `IMU` and the framework already publish diagnostics. Still missing:

- `EBC` and `Display` diagnostics.
- The real publication rate of each channel.
- HG4 motor and FET temperatures, `TractionLoss` and tick counters.
- Cell voltage, `LifeTime`, `AccuStatus` and `PowerSupplyPresent` for the battery.
- `ContactsDown` for the charger.
- `IMUGood` (HG4).
- Serial number and firmware version of each CAN module (`ModuleInfo`, `SerialNumber`, `SoftwareVersion`) as `hardware_id`.

### 4.6 Hardware error memory

- Services `~/drive/get_errors` and `~/drive/clear_errors` over `getErrorInfo`, `readoutErrorMemory` and `clearErrorMemory`. On the Morphia the methods are exposed as `charger_*` plus `getErrorDetails`.
- Service `~/charger/get_persistent_errors` over `getPersistentErrors`. Today the persistent errors can only be saved to a file on the robot (`savePersistentErrors`).

### 4.7 Charging dock contact

- Publish `ContactsDown` and `PowerSupplyPresent` from the charger, to give docking a direct confirmation of electrical contact that is more reliable than inferring it from the charging state.
- They are not channels on the Morphia: check whether they are properties.

### 4.8 Total mileage and floor RFID

- `TotalMileage` and `RFIDFloorTag` (distinct from the `RFIDUserTag` used for the magnetic barrier).
- Neither exists as a channel on the Morphia.

### 4.9 Kinematic model and footprint from `getRobotModel`

- `getRobotModel()` returns the `DifferentialRobotModel`: maximum velocities (0.8 m/s forward, -0.2 m/s backward, 120 °/s), accelerations and decelerations, wheel distance (0.376 m) and the `BumperFront` and `SafetyZone` collision polygons.
- Read the model on configure and publish it as read-only node parameters and as latched `geometry_msgs/PolygonStamped` on `footprint` and `safety_zone`.
- Warn when the Nav2 limits (velocity smoother, `robot_radius`/footprint in `morphia.yaml`) exceed the ones the driver applies.
- Prerequisite: dump the full JSON of `getRobotModel` to a file and decide whether `robot_radius` and the footprint come from the model.

### 4.10 MIRA transforms check

- Do not replace the URDF (it carries cameras and calibrations MIRA does not know about).
- Add a checking tool that compares MIRA mount frames (for example `frontLaser/MountFrame`) with the `base_link → laser` TF of the URDF and warns about differences.

### 4.11 HG4 services **[Morphia]**

- `setSoftwareSafeStop`: a software safe stop, cleaner than `emergencyStop`, which requires re-arming with `resetMotorStop`.
- `activateBumperEscapeMode`: allows reversing out after a collision without disabling the bumper.

### 4.12 `driveDistance` **[other robots]**

Short, exact movements executed by the controller itself, useful for the last stretch of docking. Only available on HG2/HG3, so it probably does not apply to the Morphia.

### 4.13 `VelocityCommandTimeout` and a `cmd_vel` watchdog **[Morphia]**

- Read and publish `VelocityCommandTimeout` (HG4).
- If the driver has no watchdog of its own, add one in `Drive` that sends zero velocity when no `cmd_vel` arrives within X ms. There is none on the ROS side today.

### 4.14 Display

Use the module's own methods (`enableUserMenu`, `setUserMenuName`, `setUserMenuEntryName`) instead of `setProperty`, and expose `LockStatusDisplay`.

## Phase 5: deployment

### 3.2 MIRA in a separate process

- The MIRA arguments from ROS (`-k`) already allow it. Missing: the systemd service and the connection mode in `MiraFramework`.
- Benefits: the robot driver survives ROS restarts (odometry is not reset), `miracenter` and the MetraLabs diagnostics tools can be used alongside ROS, and the ROS authorities show up as remote clients in the MIRA graph.
- Keep the in-process mode as the default and add the remote mode through a parameter.

### 5. Cleanup of `morphia_bringup/resources/Tory`

- Keep only `SCITOSRobotAttributes.xml` (the Morphia-specific part) and a minimal `SCITOSDriver.xml` that includes `ToryConfig:etc/Tory.xml`, so MIRA updates bring MetraLabs fixes without manual merging. `Tory.xml` and `Tory-IgnoreIntervals-SickS300.xml` are currently copies of the ones in `ToryConfig`, and the local intervals file is not used.
- Remove the `<var>` entries and `<if robot="SCITOS-…">` blocks placed after the `include` in `SCITOSDriver.xml`: the first definition of a `<var>` wins, so they are dead code.
- Calibrate the odometry: `useOdometryCorrection` is `false` with all factors at 1.0. A calibration run (`SCITOSConfigs:etc/SCITOS-calibrate.xml`) allows tuning `cb`, `cr` and `cd`, a direct improvement for localization.

## Tests and loose ends of what is already implemented

- **3.5 Simulator and tapes:** record `miratape` tapes of the real robot (including a bumper press, an emergency stop and a charge) to test the decoding on the public CI, and add an end-to-end test against the Tory simulator (`mira -v robot=Tory SCITOSSimulator:etc/SCITOSDriver.xml`). The SCITOS packages are proprietary and probably not in the public installer used by the CI, so the simulator tests would run locally or on the robot.
- **2.2 Lifecycle:** `Charger` and `Display` still do not stop forwarding data while inactive; their publishers drop the messages with a warning.
- **3.1 MIRA arguments:** `mira_variables` and `mira_known_frameworks` are untested.
- **2.1 RPC timeout:** the CAN cable cut has not been tested.
- **Phase 1:** the long-duration test is missing.
- **2.4 `RFIDReaderEnabled`:** it is still unknown what it does and whether `enable_rfid` should be separated from `magnetic_barrier_enabled`. The Morphia has no magnetic barrier, so both currently have no effect on this robot.
- **Checks before phase 4:** find the types of the properties that are not channels, and store the `mirainspect` output in the documentation repository.

## Suggested starting points

- 4.13 (`cmd_vel` watchdog).
- 4.11 (HG4 services).
- 4.6 (hardware errors; the methods already exist on the Morphia).
- 4.9 (once the model has been dumped).
