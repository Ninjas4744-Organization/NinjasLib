# Getting Started

## Installation

NinjasLib is distributed as source, not a Maven artifact — you add it to your robot project as a
git submodule and it compiles alongside your own code.

1. From your robot project's root, add the submodule under your `frc.lib` package folder:

    ```bash
    git submodule add https://github.com/Ninjas4744-Organization/NinjasLib.git src/main/java/frc/lib/NinjasLib
    ```

2. NinjasLib depends on several WPILib vendor libraries and two plain Java libraries. Make sure your
   project has these vendordeps installed (via WPILib's VS Code "Manage Vendor Libraries" command, or
   by dropping the JSON files into `vendordeps/`):

    - `WPILibNewCommands`
    - `Phoenix6` (CTRE TalonFX/Pigeon2)
    - `Phoenix5` (CTRE TalonSRX/VictorSPX) — only needed if you use those controllers
    - `REVLib` (REV SparkMax) — only needed if you use `SparkMaxController`
    - `Studica` (NavX) — only needed if you use a NavX gyro
    - `photonlib` (PhotonVision) — only needed if you use vision
    - `PathplannerLib` — only needed if you use `NinjasAutoBuilder` / swerve autonomous
    - `DogLog` — NinjasLib's logging (`NinjasLogger`) is built directly on top of this
    - [`maple-sim`](https://github.com/Shenzhen-Robotics-Alliance/maple-sim) — physics simulation
      backing `Swerve`'s simulated mode

3. NinjasLib also needs two plain Java dependencies that aren't vendordeps — add them directly to
   `build.gradle`:

    ```groovy
    dependencies {
        implementation 'org.jgrapht:jgrapht-core:1.5.2' // used by StateMachineBase
        implementation 'org.dyn4j:dyn4j:5.0.2'          // used by maple-sim
    }
    ```

4. Run a build. If anything is missing, the compiler error will point at the specific class, which
   tells you which vendordep you skipped.

## The shape of a NinjasLib-based robot

NinjasLib doesn't impose a project structure — you still write ordinary `SubsystemBase` and
`Command` classes. But its core modules (`Swerve`, `SwerveController`, `Vision`) are **singletons**
with their own `periodic()` that needs to run every loop, so the idiomatic way to use them is to
**own one inside a subsystem of your own**, rather than constructing them loose in `RobotContainer`.
Your subsystem's constructor builds and registers the singleton, and its `periodic()` drives it —
exactly like it would for a `Controller` or any other piece of hardware it owns.

Two subsystems come up on essentially every NinjasLib robot: a drivetrain subsystem wrapping
`Swerve`/`SwerveController`, and a vision subsystem wrapping `Vision`. The first is naturally a
[state machine](guides/state-machine.md) — driving means being in exactly one "mode" at a time
(driver-controlled, following an autonomous path, aiming at a target, ...) — so it's built as a
`StateMachineBase` rather than a plain `SubsystemBase`:

```java
public class SwerveSubsystem extends StateMachineBase<SwerveSubsystem.SwerveState> {
    public enum SwerveState { DRIVER, AUTO }

    private final DoubleSupplier driverLeftX, driverLeftY, driverRightX, driverRightY;
    private SwerveSpeeds autoInput = new SwerveSpeeds();

    public SwerveSubsystem(DoubleSupplier driverLeftX, DoubleSupplier driverLeftY,
                            DoubleSupplier driverRightX, DoubleSupplier driverRightY) {
        super(SwerveState.class);
        currentState = SwerveState.DRIVER;

        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;
        this.driverRightY = driverRightY;

        // This subsystem owns Swerve and SwerveController: it builds them once, here.
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
        SwerveController.get().setChannel("Driver");
    }

    @Override
    protected void define() {
        // Whichever mode is active continuously feeds SwerveController through its own channel;
        // an omni-edge lets any other mode switch into it on demand.
        addOmniEdge(SwerveState.DRIVER, () -> Commands.runOnce(() -> SwerveController.get().setChannel("Driver")));
        addStateCommand(SwerveState.DRIVER, Commands.run(() -> SwerveController.get().setControl(getDriverInput(), "Driver")));

        addOmniEdge(SwerveState.AUTO, () -> Commands.runOnce(() -> SwerveController.get().setChannel("Auto")));
        addStateCommand(SwerveState.AUTO, Commands.run(() -> SwerveController.get().setControl(autoInput, "Auto")));
    }

    private SwerveSpeeds getDriverInput() {
        return new SwerveSpeeds(driverLeftY.getAsDouble(), driverLeftX.getAsDouble(),
            driverRightX.getAsDouble(), GeneralConstants.Swerve.kDriverFieldRelative);
    }

    /** Called by PathPlanner/NinjasAutoBuilder while an autonomous path is following. */
    public void setAutoInput(ChassisSpeeds speeds) {
        autoInput = new SwerveSpeeds(speeds, false);
    }

    @Override
    public void periodic() {
        SwerveController.get().periodic(); // also drives Swerve.periodic(), which feeds RobotPose
        super.periodic();
    }
}
```

This is a deliberately trimmed-down version — see [State Machines](guides/state-machine.md) for why
this pattern (an omni-edge into a mode, plus a state command that continuously drives it) is the
right shape for "the drivetrain is always in exactly one mode," and [Swerve Drive](guides/swerve.md)
for everything `Swerve`/`SwerveController` themselves offer. A real drivetrain subsystem usually grows
several more states this way (aiming at a goal, snapping to a fixed angle, following a scoring
target, ...), all sharing the same `SwerveController` channel gate so exactly one of them drives at a
time.

`Vision` doesn't need to be a state machine — a plain `SubsystemBase` that owns it and feeds
`RobotPose` each loop is enough:

```java
public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        Vision.setInstance(new Vision(SubsystemConstants.kVision));
    }

    @Override
    public void periodic() {
        Vision.get().periodic();

        for (VisionOutput output : Vision.get().getVisionOutputs())
            RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
    }
}
```

`RobotPose` itself isn't owned by either subsystem — it has no `periodic()` of its own (`Swerve`
feeds it odometry directly from its own `periodic()`), so it's simplest to just construct it once in
`RobotContainer`, alongside your subsystems:

```java
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem(
            driverController::getLeftX, driverController::getLeftY,
            driverController::getRightX, driverController::getRightY);

        RobotPose.setInstance(new RobotPose(
            SubsystemConstants.kSwerve.chassis.kinematics,
            VisionStrengthCalculator.ninjasFunction(() -> odometryDrift),
            output -> output.ambiguity < 0.2
        ));

        visionSubsystem = new VisionSubsystem();

        // ... construct the rest of your subsystems as usual
    }
}
```

Because `SwerveSubsystem` and `VisionSubsystem` are `SubsystemBase`s (a `StateMachineBase` is one
too), WPILib's `CommandScheduler` already calls their `periodic()` every loop automatically once
they're constructed — you don't need to call `Swerve.get().periodic()`/`Vision.get().periodic()`
from `Robot.robotPeriodic()` yourself; that's exactly why each subsystem's own `periodic()` above
calls into the module it owns.

Every one of `Swerve`/`SwerveController`/`Vision`/`RobotPose`'s singletons is safe to skip entirely:
if you never call `setInstance`, calling `get()` still returns a disabled placeholder instead of
throwing, so a robot without vision, say, simply never constructs a `VisionSubsystem`, and any
accidental `Vision.get()` call elsewhere degrades gracefully instead of crashing.

## Where to go next

- Driving the robot around: [Swerve Drive](guides/swerve.md)
- Configuring a mechanism's motor(s): [Motor Controllers](guides/controllers.md)
- Modeling a subsystem's behavior as states: [State Machines](guides/state-machine.md)
- Knowing where the robot is on the field: [Localization & Vision](guides/localization.md)
