# Utilities

Smaller pieces of NinjasLib that don't need a full guide of their own, grouped here by what they do.

## `NinjasLogger`

`NinjasLogger extends DogLog`, so every [DogLog](https://github.com/DogLogger/DogLog) logging method
(`NinjasLogger.log(key, value)`) works exactly as it does in DogLog — this is what every module in
NinjasLib uses internally, and what you should use in your own subsystems for consistency. On top of
that, it adds numbered event logging:

```java
NinjasLogger.logEvent("Intake started");
// prints "[3] Intake started" to the console and logs it under "Event"
```

Each event gets a short, compact incrementing id (base-22, e.g. `A`, `B`, ..., `K`, `10`, ...) so you
can tell the order events happened in even once DogLog's own timestamps aren't precise enough to
disambiguate two events in the same loop.

```java
NinjasLogger.logEventImportant("Vision instance not set.");
```

`logEventImportant` is for problems you don't want a driver/programmer to miss during a match: it
reports the message as a `DriverStation` error (which shows up loudly in the Driver Station console)
and logs it several times in a row instead of once. This is what every disabled-singleton fallback in
the library (`Swerve.get()`, `RobotPose.get()`, `Vision.get()`, ...) uses to warn you that
`setInstance` was never called.

## `NinjasAutoBuilder`

A thin wrapper around PathPlanner's `AutoBuilder` that only exposes building/choosing autos, with two
differences from using `AutoBuilder` directly: `buildAuto` lets you choose whether to mirror the auto
to the other alliance side, and the choosers return the chosen auto's **name** rather than a built
command, so you build it yourself once autonomous actually starts (with the mirroring you want at
that moment):

```java
// Once, during setup (after PathPlanner's own AutoBuilder.configure(...) has been called):
SendableChooser<String> autoChooser = NinjasAutoBuilder.buildAutoChooser("MyDefaultAuto");
SmartDashboard.putData("Auto Chooser", autoChooser);

// In autonomousInit():
String chosenAuto = autoChooser.getSelected();
if (!chosenAuto.isEmpty()) {
    boolean shouldMirror = RobotPose.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    PathPlannerAuto auto = NinjasAutoBuilder.buildAuto(chosenAuto, shouldMirror);
    auto.schedule();
}
```

`buildAutoChooserWithOptionsModifier` additionally lets you transform the list of auto names before
they're added to the chooser (e.g. to filter out test-only autos, or reformat display names).

## `DerivativeCalculator` / `DerivativeCalculator2d`

A small stateful helper for turning a noisy sampled value into a smoothed rate of change, e.g.
estimating acceleration from a velocity signal, or velocity from a position signal, without writing
your own low-pass filter every time:

```java
private final DerivativeCalculator accelCalculator = new DerivativeCalculator(8); // average window

@Override
public void periodic() {
    double acceleration = accelCalculator.calculate(getVelocity());
}
```

It's a moving-average filter over the raw sample-to-sample derivative: a larger `averageWindow` is
smoother but laggier — 5–10 samples is a reasonable starting point. Because it keeps state between
calls (the last value/timestamp), call it once per loop with a consistent signal, and don't share one
instance across unrelated values. `DerivativeCalculator2d` is the same idea for a `Translation2d`
signal, e.g. differentiating a 2D position into a velocity vector.
