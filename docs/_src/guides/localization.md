# Localization & Vision

Two classes cooperate to answer "where is the robot on the field": **`RobotPose`** fuses swerve
odometry with vision corrections into a single pose estimate, and **`Vision`** manages any number of
cameras (Limelight or PhotonVision, real or simulated) and feeds their pose estimates into it. Both
are singletons, registered once with `setInstance` like the rest of the library.

## `RobotPose`

`RobotPose` keeps two pose trackers in lockstep:

- **`getRobotPose()`** — the vision-fused estimate. Use this everywhere by default (aiming, logging,
  autonomous).
- **`getOdometryOnlyRobotPose()`** — odometry only, ignoring vision entirely. Use this only when a
  vision correction's discontinuity would hurt you, e.g. feeding a velocity/acceleration-based
  control loop that assumes a continuous position signal. It drifts over time, since it never
  corrects against vision.

```java
RobotPose.setInstance(new RobotPose(
    SubsystemConstants.kSwerve.chassis.kinematics,
    VisionStrengthCalculator.ninjasFunction(() -> myOdometryDriftEstimate),
    output -> output.ambiguity < 0.2 && output.closestTargetDist < 4.0
));
```

You rarely call `RobotPose`'s update methods directly — `Swerve.periodic()` already calls
`addOdometryUpdate`/`addTimedOdometryUpdate` for you every loop. What you *do* wire up yourself are
the two calculators passed into the constructor, both `@FunctionalInterface`s so a lambda is enough:

- **`VisionFiltersCalculator`** — `isPassed(VisionOutput)` decides whether an incoming vision
  measurement should be rejected outright (e.g. too far away, too ambiguous, or physically
  implausible given the current pose) before it ever reaches the pose tracker.
- **`VisionStrengthCalculator`** — `calculate(VisionOutput)` returns a 3x1 matrix of per-axis
  (x, y, theta) standard deviations expressing how much to trust a measurement that *did* pass the
  filter. NinjasLib ships two ready-made ones: `VisionStrengthCalculator.kDefault` (a fixed trust for
  every measurement) and `VisionStrengthCalculator.ninjasFunction(odometryDrift)`, which weighs trust
  by distance to the closest tag and by how much your odometry has already drifted — write your own
  lambda only if neither fits.

Common queries:

```java
RobotPose.get().getDistance(targetPose);      // meters, robot to target
RobotPose.get().getTransform(targetPose);     // dx, dy, dtheta, field-relative
RobotPose.get().getRotation();                // current field-relative heading
RobotPose.get().setRobotPose(knownPose);      // e.g. at auto start, from a known starting position
RobotPose.get().resetGyro(Rotation2d.kZero);  // re-zero heading in place, keeping current translation
```

## `Vision`

`Vision` owns every camera described in a `VisionConstants` object and polls all of them once per
loop:

```java
public static final VisionConstants kVision = new VisionConstants()
    .withFieldLayoutGetter(ignoredTags -> Optional.of(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape)))
    .withRobotPoseSupplier(() -> RobotPose.get().getRobotPose())
    .withPhotonVision("FrontCam", new Transform3d(/* robot-to-camera offset */))
    .withLimelight("limelight-back");

Vision.setInstance(new Vision(kVision));
```

If `cameras`, `fieldLayoutGetter`, or `robotPoseSupplier` is missing, `Vision` disables itself
entirely rather than partially starting — every accessor then returns a harmless default and
`periodic()` is a no-op, so a robot that hasn't wired up vision yet doesn't need special-casing
elsewhere. In simulation, `Vision` also builds a `VisionSystemSim` seeded from `fieldLayoutGetter` and
advances it from `robotPoseSupplier` each loop, so simulated cameras see AprilTags consistent with the
robot's simulated pose.

```java
@Override
public void robotPeriodic() {
    Vision.get().periodic(); // the only place cameras are actually polled
}
```

### Feeding vision into `RobotPose`

The usual pattern is: read each camera's output from `Vision`, then hand it to `RobotPose`:

```java
for (VisionOutput output : Vision.get().getVisionOutputs()) {
    RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
}
```

`addVisionUpdate` does nothing if the output reports no targets; otherwise it runs your
`VisionFiltersCalculator`, and if the measurement passes, applies it through your
`VisionStrengthCalculator`'s computed trust. Both the pass/fail result and the computed strength are
logged per-camera regardless of outcome, so a rejected or heavily-discounted measurement is still
visible in your logs.

If you've already decided a pose is trustworthy through some other means, `addManualVisionUpdate`
applies it directly, bypassing both calculators.

### Other per-camera queries

```java
Vision.get().hasTargets("FrontCam");
Vision.get().getClosestTargetDistance("FrontCam");
Vision.get().getCameraToClosestTargetTransform("FrontCam");
Vision.get().ignoreTag(4);    // stop trusting a specific AprilTag (e.g. a known-bad/moved tag)
Vision.get().unIgnoreTag(4);
```

## Putting it together

```java
public class RobotContainer {
    public RobotContainer() {
        Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        RobotPose.setInstance(new RobotPose(
            SubsystemConstants.kSwerve.chassis.kinematics,
            new MyVisionStrengthCalculator(),
            new MyVisionFiltersCalculator()));
        Vision.setInstance(new Vision(SubsystemConstants.kVision));
    }
}

// Robot.java
@Override
public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Swerve.get().periodic();  // feeds odometry into RobotPose
    Vision.get().periodic();  // polls cameras

    for (VisionOutput output : Vision.get().getVisionOutputs())
        RobotPose.get().addVisionUpdate(output, Timer.getFPGATimestamp());
}
```
