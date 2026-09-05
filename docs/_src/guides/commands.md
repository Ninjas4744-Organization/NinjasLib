# Commands

NinjasLib ships a handful of small `Command`/helper classes that come up often enough to be worth
sharing, rather than any single subsystem depending on a large command framework of its own.

## `StateEndCommand`

Behaves like WPILib's `SequentialCommandGroup`, with one difference: once finished, `isFinished()`
keeps returning `true` forever instead of only until the group is re-run. This is what
[`StateMachineBase.addStateEnd`](state-machine.md) expects, since it polls `isFinished()` once per
loop and needs a stable answer instead of one that could flip back to `false`.

```java
addStateEnd(State.INTAKING,
    new StateEndCommand(Commands.waitUntil(beamBreak::get), Commands.waitTime(Seconds.of(0.1))),
    State.HOLDING);
```

The commands passed in should only represent *waiting for an event* (`waitUntil`, `waitTime`, and
similar) — not real subsystem logic, since a state-end command's lifecycle is driven by the state
machine rather than the normal command scheduler flow.

## `BackgroundCommand`

Not itself a `Command` — a small slot that holds at most one running command, automatically
cancelling whatever was running when you assign a new one. This is what
[`StateMachineBase`](state-machine.md) uses internally for `addStateCommand`, but it's equally useful
any time you need "run this task for now, but starting a new one should cancel the old one":

```java
private final BackgroundCommand idleTask = new BackgroundCommand();

// Somewhere in your code:
idleTask.setNewTask(Commands.run(() -> motor.setPercent(0.05)));

// Later, replacing it automatically cancels the previous task:
idleTask.setNewTask(Commands.run(() -> motor.setPercent(0)));

idleTask.isRunning(); // whether the current task is still scheduled and not finished
idleTask.stop();      // cancel without replacing
```

`setNewTaskCommand(task)` / `setNewTaskDynamic(supplier)` wrap the same behavior as an `InstantCommand`
for binding to a trigger.

## `DetachedCommand`

An `InstantCommand` that hands another command off to the scheduler as its own independent command,
instead of running it inline. Use it when a command needs `requirements` for sequencing purposes, but
the actual work should keep running after the wrapping command (or sequence) ends:

```java
Commands.sequence(
    prepCommand,
    new DetachedCommand(longRunningBackgroundCommand, subsystem)
    // the sequence "finishes" here; longRunningBackgroundCommand keeps running independently
);
```

## `LoopCommand`

Wraps a command and restarts it (`end()` then `initialize()`) every time it finishes, for a fixed
number of repeats, sharing a single command instance across every repeat rather than creating new
ones:

```java
Command blinkTwice = new LoopCommand(
    Commands.sequence(Commands.runOnce(this::toggleLED), Commands.waitTime(Seconds.of(0.1))),
    4 // toggle on, off, on, off
);
```
