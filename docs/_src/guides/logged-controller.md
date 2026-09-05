# Gamepad Input (LoggedCommandController)

`LoggedCommandController` wraps a PS5 or Xbox gamepad to give you two things WPILib's own
`CommandPS5Controller`/`CommandXboxController` don't: every button, POV direction, and axis is
automatically logged via `NinjasLogger` each loop, and the PS5/Xbox difference is hidden behind one
shared API (`cross()`/`circle()`/... map to `A()`/`B()`/... automatically depending on which IO you
give it).

## Setup

```java
LoggedCommandController driver = new LoggedCommandController(
    "DriverController",
    new LoggedCommandControllerIOPS5(0)  // or LoggedCommandControllerIOXbox(port) for an Xbox pad
);
```

The `name` argument is the DogLog key prefix (`"DriverController/cross"`,
`"DriverController/leftX"`, ...); call `periodic()` once per loop, typically alongside your other
controller polling in `robotPeriodic()`:

```java
@Override
public void robotPeriodic() {
    driver.periodic();
}
```

## Binding buttons

Every button/POV direction is exposed as a WPILib `Trigger`, so it binds exactly like a normal
`CommandXboxController` button:

```java
driver.cross().onTrue(indexer.changeStateCommand(Indexer.State.INTAKING));
driver.R1().whileTrue(shooterCommand);
driver.povUp().onTrue(Commands.runOnce(() -> RobotPose.get().resetGyro(Rotation2d.kZero)));
```

Face buttons are named both ways (`cross()`/`circle()`/`square()`/`triangle()` and
`A()`/`B()`/`X()`/`Y()`) so you can use whichever naming matches the physical controller your drive
team actually has, regardless of which `IO` you constructed it with.

## Reading axes

Axes are plain `double` getters instead of `Trigger`s, in `[-1, 1]` for sticks and `[0, 1]` for the
analog triggers:

```java
double forward = -driver.getLeftY();
double strafe  = -driver.getLeftX();
double rotate  = -driver.getRightX();
double intakePower = driver.getR2Axis();
```

## Which button maps to what

| Category | Methods |
|---|---|
| Face buttons | `cross()`/`A()`, `circle()`/`B()`, `square()`/`X()`, `triangle()`/`Y()` |
| D-Pad | `povUp()`, `povDown()`, `povLeft()`, `povRight()` |
| Bumpers / stick clicks | `L1()`, `R1()`, `L3()`, `R3()` |
| Triggers as buttons | `L2()`, `R2()` (digital threshold) |
| System buttons | `create()`/back, `options()`/start, `ps()` (PS5 only), `touchpad()` (PS5 only) |
| Axes | `getLeftX()`, `getLeftY()`, `getRightX()`, `getRightY()`, `getL2Axis()`, `getR2Axis()` |

`ps()` and `touchpad()` are always inactive on an Xbox controller (there's no hardware button to read),
so it's safe to bind them unconditionally even in code shared between a PS5 and Xbox setup.
