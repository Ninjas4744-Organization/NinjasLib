# NinjasLib

NinjasLib is Team 4744's shared FRC library: a set of ready-made building blocks for swerve
drivetrains, motor control, robot pose estimation, vision, state machines, and other patterns that
show up in almost every robot codebase. Instead of re-solving the same problems every season, teams
using NinjasLib configure these modules for their robot's hardware and get a tested, simulation-aware
implementation for free.

This site is the guide layer on top of the library: conceptual explanations, usage patterns, and
full code examples for each module. For the exhaustive, auto-generated method-by-method reference,
use the **Javadoc** link in the navigation bar.

## Where to start

- New to NinjasLib? Start with [Getting Started](getting-started.md) for installation and the
  overall shape of a robot project built on it.
- Already have it installed? Jump straight to the guide for the module you need:

| Module | What it's for |
|---|---|
| [Swerve Drive](guides/swerve.md) | Driving a 4-module swerve drivetrain, in both real hardware and simulation |
| [Motor Controllers](guides/controllers.md) | A hardware-agnostic wrapper around TalonFX/SparkMax/TalonSRX/VictorSPX motors |
| [State Machines](guides/state-machine.md) | Modeling a subsystem (or the whole robot) as states and transitions |
| [Localization & Vision](guides/localization.md) | Tracking the robot's field pose and fusing in AprilTag vision |
| [Gamepad (LoggedCommandController)](guides/logged-controller.md) | A logged wrapper around PS5/Xbox controllers |
| [Logged Digital Input](guides/logged-digital-input.md) | A logged, optionally-disabled digital sensor wrapper |
| [Commands](guides/commands.md) | Small `Command` building blocks used throughout the library |
| [Utilities](guides/utilities.md) | Autonomous building (PathPlanner) and event logging |

## Design philosophy

A few patterns repeat across every module in NinjasLib, so it's worth calling them out once instead
of in every guide:

- **Singletons with a disabled fallback.** Most top-level classes (`Swerve`, `RobotPose`, `Vision`,
  `SwerveController`, ...) are accessed through a static `get()` after being installed once with
  `setInstance(...)`. If you never call `setInstance`, `get()` still returns a working object — just
  one where every method is a safe no-op (returning zero/identity values) and logs a warning. This
  means you can leave a module unused on a robot without null-checking it everywhere.
- **Real vs. simulated is handled inside the constructor.** Classes like `Swerve` and `Controller`
  branch on `Robot.isReal()` internally and build the right hardware or simulation backend. Your
  subsystem code doesn't need `if (Robot.isReal())` checks of its own for these modules.
- **Everything logs itself.** Modules push their state to [DogLog](https://github.com/DogLogger/DogLog) via `NinjasLogger` automatically from their own `periodic()`. You get telemetry for free as long as you call `periodic()` — no separate logging code needed.
- **Configuration is a builder-style constants object.** Modules are configured through a plain
  object with public fields and chainable `withX(...)` setters (e.g. `SwerveConstants`,
  `RealControllerConstants`), not constructor argument lists. Set the fields you care about and leave
  the rest at their defaults.
