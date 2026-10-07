# Swerve Drive

The swerve module is two cooperating classes:

- **`Swerve`** — the drivetrain itself. Owns the four modules and the gyro, applies acceleration and
  speed limiting, and feeds odometry. Built once from a `SwerveConstants` object.
- **`SwerveController`** — a thin layer on top of `Swerve` that adds rotation/translation PID
  ("look at this point", "drive to this point") and a shared-access gate (`setControl`/`setChannel`)
  so multiple commands can't fight over the drivetrain at once.

Both are singletons: build them once, register with `setInstance`, and access them everywhere else
via `Swerve.get()` / `SwerveController.get()`.

## Where each piece of code goes

Every snippet on this page lives in one of two places. The constants go in your robot's constants
file, and everything that calls into `Swerve`/`SwerveController` goes in your drivetrain subsystem:

| Snippet | Where it goes |
| --- | --- |
| `kDriveMotorConstants`, `kSteerMotorConstants`, `kSwerve`, `kSwerveController` | Your constants file, e.g. `frc/robot/constants/SubsystemConstants.java`. Declared as `public static final` fields, in that order. |
| `Swerve.setInstance(...)`, `SwerveController.setInstance(...)` | The **constructor** of your drivetrain subsystem (e.g. `SwerveSubsystem`), `Swerve` first. The subsystem itself is constructed once, in `RobotContainer`. |
| `Swerve.get().drive(...)`, `SwerveController.get().setControl(...)`, `lookAt`/`pidTo` | Your drivetrain subsystem's `periodic()`, or a command / state command it owns — anywhere that runs every loop **after** the constructor ran. |
| `Swerve.get().periodic()` / `SwerveController.get().periodic()` | The drivetrain subsystem's `periodic()`. Nothing else calls it for you. |
| Reading state (`getGyro()`, `getSpeeds()`, ...) | Anywhere, once the instances exist. |

[Getting Started](../getting-started.md#the-shape-of-a-ninjaslib-based-robot) shows a complete
drivetrain subsystem with all of the above in place.

## Configuring the drive and steer motors

Each module has one drive motor (spins the wheel) and one steer motor (turns the module). All four
drive motors share one `ControllerConstants` object, and all four steer motors share another — here
`kDriveMotorConstants` and `kSteerMotorConstants`. They're ordinary
[motor controller constants](controllers.md), so everything there applies; this section covers what's
specific to swerve.

**You don't set the motor IDs or inversions here.** Those differ per module, so they come from
`SwerveModuleConstants` (see below); the library clones the shared constants for each module and
fills in that module's IDs and inversion. The CAN bus comes from `special.CANBus`, and the module's
CANcoder is configured by the module constants too, so leave `canCoder`, followers and soft/hard
limits alone.

What you *do* set:

- **`control.gearRatio` and `control.conversionFactor`** decide the units the library works in, and
  they must match what `Swerve` expects:
    - **Drive:** meters and meters per second. `gearRatio` is your module's drive reduction (e.g.
      `6.75` for an SDS MK4i L2) and `conversionFactor` is the wheel circumference,
      `2 * Math.PI * wheelRadiusMeters`.
    - **Steer:** radians. `gearRatio` is your module's steer reduction (e.g. `150.0 / 7`
      for an MK4i) and `conversionFactor` is `2 * Math.PI`.
- **`control.controlConstants`** are the gains. The drive motor runs closed-loop *velocity* control,
  so use `ControlConstants.createPIDF(...)` with a velocity feed-forward `V` of roughly
  `12 / maxSpeed` (volts per m/s) and a small static-friction `S`; `A` and `G` are `0`, and the
  `GravityTypeValue` argument is required but has no effect on a drivetrain. The steer motor runs
  closed-loop *position* control, so a plain `ControlConstants.createPID(P, I, D, IZone)` is enough.
  The library handles wrapping around ±180° itself, so you don't configure continuous input.
- **`base` current limits** — the drive motor needs a higher stator limit (it supplies the torque
  to accelerate the robot) than the steer motor does.
- **The controller type** — `Controller.ControllerType.TalonFX` etc. — is *not* part of the
  constants; you pass it next to them in `withDriveMotor(...)`/`withSteerMotor(...)`. The drive and
  steer motors may be different types, but the high-frequency odometry thread
  (`special.enableOdometryThread`) requires both to be TalonFX.

```java
public class SubsystemConstants {
    private static final double kWheelRadius = 0.049; // meters

    // Declare these *above* kSwerve: static fields initialize top to bottom, so a field that
    // is defined further down would still be null when kSwerve reads it.
    public static final ControllerConstants kDriveMotorConstants = new ControllerConstants();
    public static final ControllerConstants kSteerMotorConstants = new ControllerConstants();

    static {
        // Drive: velocity control, in meters and meters per second
        kDriveMotorConstants.real
            .withBase(new RealControllerConstants.Base()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(60))
            .withControl(new RealControllerConstants.Control()
                .withConversion(6.75, 2 * Math.PI * kWheelRadius)     // gear ratio, circumference
                .withControlConstants(ControlConstants.createPIDF(
                    1.0, 0, 0, 0,                                     // P, I, D, IZone
                    2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static))); // V, A, S, G, gravity type

        // Steer: position control, in radians
        kSteerMotorConstants.real
            .withBase(new RealControllerConstants.Base()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30))
            .withControl(new RealControllerConstants.Control()
                .withConversion(150.0 / 7, 2 * Math.PI)               // gear ratio, radians per rotation
                .withControlConstants(ControlConstants.createPID(25, 0, 0.25, 0)));
    }

    // public static final SwerveConstants kSwerve = ... (next section)
}
```

In simulation only the `P`, `I`, `D` and `IZone` gains are read from these objects. The physical
model comes from `SwerveConstants.simulation`, so the gear ratio and conversion factor above are
ignored there — set them for the real robot, then tune the gains separately if the sim drives
differently.

## Configuring `SwerveConstants`

`SwerveConstants` groups configuration into nested objects, each with chainable `withX(...)` setters.
You only need to override what differs from the defaults. This goes in the same constants file, below
the motor constants it references:

```java
public class SubsystemConstants {
    // kDriveMotorConstants and kSteerMotorConstants from above

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
  steerInverted, canCoderID, canCoderInverted, canCoderOffset)`. The IDs and inversions here are what
  each module's motors actually use (they override the placeholders in the shared motor constants),
  and the offset is the CANcoder's magnet offset in rotations, calibrated so the module reads zero
  when the wheel points forward. The module order (indices 0–3) must match the corner order
  used by `chassis.kinematics`: front left, front right, back left, back right.
- **`withSimulation(...)`** (not shown above) sets the simulated module type, e.g.
  `new SwerveConstants.Simulation().withSwerveType(SwerveConstants.Simulation.SwerveType.Mark4i, 2)`; only used in simulation,
  and the defaults are fine until you want the sim to match your actual modules.
- **`special.enableOdometryThread`** (via `.withOdometryThread(frequency)`) runs odometry sampling on
  a dedicated high-frequency thread instead of the main 50&nbsp;Hz loop — useful if you need
  finer-grained pose updates for something like a shoot-on-the-move calculation. Leave it off unless
  you have a specific reason to enable it.
- **`special.enableAutoLock`** (via `.withAutoLock(frames)`) makes `drive()` automatically lock the
  wheels into an X pattern after the driver holds zero input for that many frames, to resist being
  pushed. Off by default.

## Building and registering `Swerve`

Do this once, in the constructor of your drivetrain subsystem, before anything calls `Swerve.get()`:

```java
public class SwerveSubsystem extends SubsystemBase {
    public SwerveSubsystem() {
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        // SwerveController.setInstance(...) goes right after this, see below
    }
}
```

The constructor inspects `Robot.isReal()` and builds either real `SwerveModuleIOReal` modules with a
`GyroIONavX`/`GyroIOPigeon2`, or a MapleSim `SwerveDriveSimulation` with simulated modules and gyro —
your code never needs to branch on real-vs-sim itself.

## Driving

Every loop, feed the wanted motion into `drive()` as a `SwerveSpeeds` — the library's own
`ChassisSpeeds` subclass that additionally carries a `fieldRelative` flag. This goes in the drivetrain
subsystem's `periodic()` (this is the simplest form; the state-machine version in
[Getting Started](../getting-started.md) calls `SwerveController.get().setControl(...)` from a state
command instead):

```java
public class SwerveSubsystem extends SubsystemBase {
    // ...constructor from above...

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

Other useful entry points, callable from any command or subsystem method:

```java
Swerve.get().stop();              // empty drive request
Swerve.get().lockWheelsToX();     // lock wheels in an X, e.g. to resist being pushed
Swerve.get().getSpeeds();         // current speed from odometry
Swerve.get().getModuleStates();   // per-module state, for diagnostics/logging
```

## Aiming and driving to a point with `SwerveController`

`SwerveController` wraps a rotation PID (plain or motion-profiled, depending on whether you configure
cruise velocity/acceleration) and a translation PID, so commands don't each reimplement "turn to face
X" or "drive towards X". Its constants go in your constants file, *below* `kSwerve`, and it's
registered in the drivetrain subsystem's constructor right after `Swerve`:

```java
// SubsystemConstants.java
public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants()
    .withSwerveConstants(kSwerve)
    .withRotationPID(ControlConstants.createPID(4.0, 0, 0.1, 0))
    .withDrivePID(ControlConstants.createPID(3.0, 0, 0, 0));

// SwerveSubsystem.java, in the constructor
Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
```

`rotationPIDConstants` becomes a *motion-profiled* PID automatically if you also set
`cruiseVelocity`/`acceleration` (e.g. via `ControlConstants.createProfiledPIDF(...)`) — otherwise it's
a plain continuous-input `PIDController`.

Then use it from the drivetrain subsystem's `periodic()`, or from a command/state command that runs
every loop:

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
system alone. Call these from the commands that want to drive — `setChannel` once when a command
starts (e.g. in `initialize()`, or the omni-edge into a state), `setControl` every loop while it runs:

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

From anywhere in your code, once the drivetrain subsystem has been constructed:

```java
Swerve.get().getGyro().getYaw();      // current gyro heading
Swerve.get().getOdometryTwist();      // translation since the last call to this method
RobotPose.get().getRobotPose();       // full field pose (see Localization & Vision)
```

See [Localization & Vision](localization.md) for how `Swerve`'s odometry output turns into a field
pose, and how vision corrects it.
