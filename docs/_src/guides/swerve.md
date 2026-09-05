# Swerve Drive

The swerve module is two cooperating classes:

- **`Swerve`** — the drivetrain itself. Owns the four modules and the gyro, applies acceleration and
  speed limiting, and feeds odometry. Built once from a `SwerveConstants` object.
- **`SwerveController`** — a thin layer on top of `Swerve` that adds rotation/translation PID
  ("look at this point", "drive to this point") and a shared-access gate (`setControl`/`setChannel`)
  so multiple commands can't fight over the drivetrain at once.

Both are singletons: build them once, register with `setInstance`, and access them everywhere else
via `Swerve.get()` / `SwerveController.get()`.

## Configuring `SwerveConstants`

`SwerveConstants` groups configuration into nested objects, each with chainable `withX(...)` setters.
You only need to override what differs from the defaults:

```java
public class SubsystemConstants {
    public static final SwerveConstants kSwerve = new SwerveConstants()
        .withChassis(new SwerveConstants.Chassis()
            .withDimensions(0.6, 0.6)      // track width, wheel base (meters)
            .withBumper(0.9, 0.9))         // bumper width, length (meters)
        .withSpeeds(new SwerveConstants.Speeds()
            .withMaxSpeeds(5.0, 9.0)                       // physical max: m/s, rad/s
            .withSpeedLimits(4.0, 7.0)                     // soft driving limits: m/s, rad/s
            .withAccelerationLimits(20, 12, 15))           // skid, forward, rotation accel
        .withModules(new SwerveConstants.Modules()
            .withModuleConstants(new SwerveModuleConstants[] {
                new SwerveModuleConstants(0, 1, 2, false, true, 3, false, 0.421),  // front left
                new SwerveModuleConstants(1, 4, 5, false, true, 6, false, 0.128),  // front right
                new SwerveModuleConstants(2, 7, 8, false, true, 9, false, 0.355),  // back left
                new SwerveModuleConstants(3, 10, 11, false, true, 12, false, 0.009) // back right
            })
            .withDriveMotor(kDriveMotorConstants, Controller.ControllerType.TalonFX)
            .withSteerMotor(kSteerMotorConstants, Controller.ControllerType.TalonFX))
        .withGyro(new SwerveConstants.Gyro(5, false, SwerveConstants.Gyro.GyroType.Pigeon2))
        .withSpecial(new SwerveConstants.Special()
            .withRobotConfig(RobotConfig.fromGUISettings())
            .withRobotStartPose(new Pose2d(3, 3, Rotation2d.kZero)));
}
```

A few fields deserve a closer look:

- **`speeds.maxSpeed` / `maxAngularVelocity`** are the drivetrain's true physical capability — used
  for desaturating module speeds. **`speedLimit` / `rotationSpeedLimit`** are the *driving* limits
  `drive()` actually enforces, which can be lower (e.g. a "slow mode"). Change them at runtime with
  `Swerve.get().setSpeedLimit(...)` / `setRotationSpeedLimit(...)`.
- **`maxSkidAcceleration` / `maxForwardAcceleration` / `rotationAccelerationLimit`** default to
  infinite (no limiting). Set them once you've characterized your robot, or leave them off if you
  don't need acceleration limiting.
- **`SwerveModuleConstants`** takes `(moduleNumber, driveMotorID, steerMotorID, driveInverted,
  steerInverted, canCoderID, canCoderInverted, canCoderOffset)`. The module order (indices 0–3) must
  match the corner order used by `chassis.kinematics`.
- **`special.enableOdometryThread`** (via `.withOdometryThread(frequency)`) runs odometry sampling on
  a dedicated high-frequency thread instead of the main 50&nbsp;Hz loop — useful if you need
  finer-grained pose updates for something like a shoot-on-the-move calculation. Leave it off unless
  you have a specific reason to enable it.
- **`special.enableAutoLock`** (via `.withAutoLock(frames)`) makes `drive()` automatically lock the
  wheels into an X pattern after the driver holds zero input for that many frames, to resist being
  pushed. Off by default.

## Building and registering `Swerve`

```java
Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
```

The constructor inspects `Robot.isReal()` and builds either real `SwerveModuleIOReal` modules with a
`GyroIONavX`/`GyroIOPigeon2`, or a MapleSim `SwerveDriveSimulation` with simulated modules and gyro —
your code never needs to branch on real-vs-sim itself.

## Driving

Every loop, feed the wanted motion into `drive()` as a `SwerveSpeeds` — the library's own
`ChassisSpeeds` subclass that additionally carries a `fieldRelative` flag:

```java
public class SwerveSubsystem extends SubsystemBase {
    @Override
    public void periodic() {
        SwerveSpeeds driverInput = new SwerveSpeeds(
            forwardMetersPerSecond,
            strafeMetersPerSecond,
            rotationRadiansPerSecond,
            /* fieldRelative = */ true
        );

        Swerve.get().drive(driverInput);
        Swerve.get().periodic(); // must be called every loop
    }
}
```

`drive()` does not apply your requested speeds directly — it treats them as a *target* that internal
state accelerates towards, clamped by the acceleration and speed limits from `SwerveConstants`. This
is what makes the drivetrain feel smooth even with a twitchy joystick input. On a real robot, the
final chassis speeds are also discretized (`ChassisSpeeds.discretize`) to correct for the 20&nbsp;ms
command loop.

Other useful entry points:

```java
Swerve.get().stop();              // empty drive request
Swerve.get().lockWheelsToX();     // lock wheels in an X, e.g. to resist being pushed
Swerve.get().getSpeeds();         // current speed from odometry
Swerve.get().getModuleStates();   // per-module state, for diagnostics/logging
```

## Aiming and driving to a point with `SwerveController`

`SwerveController` wraps a rotation PID (plain or motion-profiled, depending on whether you configure
cruise velocity/acceleration) and a translation PID, so commands don't each reimplement "turn to face
X" or "drive towards X":

```java
public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants()
    .withSwerveConstants(kSwerve)
    .withRotationPID(ControlConstants.createPID(4.0, 0, 0.1, 0))
    .withDrivePID(ControlConstants.createPID(3.0, 0, 0, 0));

SwerveController.setInstance(new SwerveController(kSwerveController));
```

`rotationPIDConstants` becomes a *motion-profiled* PID automatically if you also set
`cruiseVelocity`/`acceleration` (e.g. via `ControlConstants.createProfiledPIDF(...)`) — otherwise it's
a plain continuous-input `PIDController`.

```java
// Face a fixed field-relative angle:
double omega = SwerveController.get().lookAt(Rotation2d.fromDegrees(90));

// Face a field-relative target pose (e.g. the speaker), with an offset if the
// mechanism that needs to aim isn't the robot's front:
double omegaAtTarget = SwerveController.get().lookAt(targetPose, Rotation2d.kZero);

// Drive straight towards a point:
Translation2d velocityToTarget = SwerveController.get().pidTo(targetTranslation);

Swerve.get().drive(new SwerveSpeeds(velocityToTarget, omegaAtTarget, true));
```

`lookAt`/`pidTo` only *calculate* a velocity — they don't call `Swerve.get().drive()` themselves, so
you can combine a rotation PID output with driver-controlled translation, or vice versa, in the same
`SwerveSpeeds`.

### Sharing the drivetrain safely between commands

When multiple commands might want to drive the swerve (driver control, auto-aim, a scoring sequence),
`setControl`/`setChannel` acts as a simple ownership gate instead of relying on WPILib's requirement
system alone:

```java
// Whichever command should currently be allowed to drive claims the channel:
SwerveController.get().setChannel("AutoAim");

// Any code path can now try to drive — only the current channel owner's call has effect:
SwerveController.get().setControl(speeds, "AutoAim"); // applied
SwerveController.get().setControl(speeds, "Teleop");  // silently ignored
```

This is most useful when a background/state-machine-owned command needs to *sometimes* take over
driving without unbinding the driver's default command.

## Reading state back

```java
Swerve.get().getGyro().getYaw();      // current gyro heading
Swerve.get().getOdometryTwist();      // translation since the last call to this method
RobotPose.get().getRobotPose();       // full field pose (see Localization & Vision)
```

See [Localization & Vision](localization.md) for how `Swerve`'s odometry output turns into a field
pose, and how vision corrects it.
