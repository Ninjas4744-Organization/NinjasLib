# Motor Controllers

`Controller` is a hardware-agnostic wrapper around a single motor mechanism (with optional
followers): percent-output, position, and velocity control, plus encoder/current/limit-switch
readback. Write your subsystem against `Controller`'s API once, and the same code drives a real
TalonFX, SparkMax, TalonSRX, VictorSPX, or a pure-software simulation, depending only on which
`ControllerType` you pick and whether `Robot.isReal()`.

## Creating a controller

Always go through the factory method rather than constructing a subclass directly — it's what
switches between real hardware and `SimulatedController` for you:

```java
Controller elevatorMotor = Controller.createController(
    Controller.ControllerType.TalonFX,
    SubsystemConstants.kElevatorMotor
);
```

## Configuring `ControllerConstants` / `RealControllerConstants`

`ControllerConstants` wraps two variants of configuration — real hardware
(`RealControllerConstants`) and simulation — but in practice you configure the `real` side and the
simulated controller reuses the relevant parts of it:

```java
public static final ControllerConstants kElevatorMotor = new ControllerConstants();
static {
    kElevatorMotor.real
        .withBase(new RealControllerConstants.Base()
            .withMain(new RealControllerConstants.Base.SimpleControllerConstants(20, false))
            .withFollowers(new RealControllerConstants.Base.SimpleControllerConstants[] {
                new RealControllerConstants.Base.SimpleControllerConstants(21, true)
            })
            .withIsBrakeMode(true)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(80))
        .withControl(new RealControllerConstants.Control()
            .withControlConstants(ControlConstants.createProfiledPIDF(
                4.0, 0, 0, 0,       // P, I, D, IZone
                80, 160, 0,         // cruise velocity, acceleration, jerk
                0, 0.4, 0.1, 0.3, GravityTypeValue.Elevator_Static))
            .withConversion(12.0, 1)          // gear ratio, then rotations -> output units
            .withPositionGoalTolerance(0.02))
        .withSoftLimits(new RealControllerConstants.SoftLimits().withMin(0).withMax(1.6))
        .withHardLimits(new RealControllerConstants.HardLimits.HardLimit[] {
            new RealControllerConstants.HardLimits.HardLimit()
                .withId(0)
                .withDirection(-1)
                .withHomePosition(0)
        });
}
```

Key fields:

- **`control.gearRatio` / `control.conversionFactor`** turn raw motor rotations into mechanism units:
  `output = motorRotations / gearRatio * conversionFactor`. If you want an elevator's `getPosition()`
  in meters, `conversionFactor` is the drum circumference; if you want an arm in degrees,
  `conversionFactor` is `360`.
- **`control.controlConstants`** is built with one of `ControlConstants.createPID(...)`,
  `createPIDF(...)`, `createProfile(...)`, or `createProfiledPIDF(...)`, matching the closed-loop
  behavior you want (plain PID vs. motion-profiled, with or without feedforward).
- **`hardLimits.limits`** describes limit switches — real ones wired to a `DigitalInput`, or
  *virtual* ones (`.withVirtual(true, stallCurrentThreshold)`) inferred purely from stator current, for
  mechanisms without a physical switch. Each limit has a `limitTriggerMethod` (defaults to
  "re-zero the encoder and hold position") that fires automatically from `Controller.periodic()`
  when the limit newly triggers.
- **`canCoder`** (via `.withId(...)` on `RealControllerConstants#canCoder`) attaches a CTRE CANcoder
  for absolute position feedback, in one of three modes — `Normal` (readable/resettable from code),
  `Fused`, or `Sync` (see the Javadoc on `CANCoderMode` for the tradeoffs between them).

## Driving the mechanism

```java
public class ElevatorSubsystem extends SubsystemBase {
    private final Controller motor = Controller.createController(
        Controller.ControllerType.TalonFX, SubsystemConstants.kElevatorMotor);

    public void setPercent(double percent) { motor.setPercent(percent); }
    public void setHeight(double meters)    { motor.setPosition(meters); }
    public boolean atGoal()                 { return motor.atGoal(); }

    @Override
    public void periodic() {
        motor.periodic(); // debounces limit switches and fires limitTriggerMethod
    }
}
```

`setPercent`/`setPosition`/`setVelocity` each switch the controller's internal `ControlState`; the
concrete subclass (e.g. `TalonFXController`) drives the actual PID/Motion Magic/feedforward using the
gains from `RealControllerConstants.Control`. `atGoal()` compares the current position or velocity
against the configured tolerance depending on which control mode is active — it always returns
`false` while in percent-output mode.

## Reading state back

```java
motor.getPosition();       // mechanism units, per conversionFactor
motor.getVelocity();       // mechanism units per second
motor.getSupplyCurrent();  // amps drawn from the battery/bus
motor.getStatorCurrent();  // amps through the motor windings
motor.getLimit();          // true if any configured limit switch is active
motor.getLimit(0);         // true if limit switch index 0 specifically is active
```

For telemetry, `motor.getLogs()` snapshots every one of these into a single struct-serializable
`Controller.ControllerLogs` object suitable for logging once per loop.

## Homing against a limit switch

The default `limitTriggerMethod` already re-zeros the encoder to `homePosition` the first time a
limit becomes active, and holds the mechanism there while it's still being driven into the limit —
which covers most "drive down until the limit switch clicks, then that's zero" homing routines with
no extra code. Override `withLimitTriggerMethod(...)` only if you need custom behavior (e.g. also
switching the subsystem's own state) when a limit triggers.
