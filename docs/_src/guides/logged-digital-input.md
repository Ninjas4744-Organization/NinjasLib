# Logged Digital Input

`LoggedDigitalInput` wraps a simple digital sensor — a limit switch, beam break, etc. — adding
automatic `NinjasLogger` logging and a built-in enable/disable fallback, so an optional sensor that
might not be wired up on a given robot doesn't need an `if (hasSensor)` check at every call site.

## Setup

```java
LoggedDigitalInput beamBreak = new LoggedDigitalInput(
    "Indexer/BeamBreak",
    0,                          // DIO port
    kHasBeamBreak,              // enabled?
    /* disabledValue = */ false,
    /* inverted = */ true,
    Robot.isReal() ? new LoggedDigitalInputIOReal() : new LoggedDigitalInputIOSim()
);
```

If `enabled` is `false`, the hardware is never touched at all — `io` is unused, and `get()` always
returns `disabledValue` instead. This is the pattern to use for a sensor that exists on some robots on
your team but not others: keep the call sites identical and flip one constant per robot.

## Usage

```java
@Override
public void periodic() {
    beamBreak.periodic(); // polls hardware (if enabled) and logs the result
}

public boolean hasPiece() {
    return beamBreak.get();
}
```

This pairs naturally with [state machines](state-machine.md): a beam break is a common
`addStateEnd` condition (`addStateEnd(State.INTAKING, beamBreak::get, State.HOLDING)`).
