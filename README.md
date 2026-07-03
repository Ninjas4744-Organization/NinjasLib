# NinjasLib

**NinjasLib** is an FRC (*FIRST* Robotics Competition) library built by **Ninjas #4744** on top of **WPILib**. It provides a reusable, opinionated set of building blocks — swerve drive, vision-based localization, motor controller wrappers, a state machine, logged I/O layers, and utility commands — so that a new season's robot code can be assembled quickly instead of rewritten from scratch every year.

The library is written entirely in Java and is designed to be dropped straight into a WPILib robot project as source code (rather than consumed as a compiled vendor dependency), living under the `frc.lib.NinjasLib` package.

## What it is

Every FRC season involves rebuilding the same core systems: swerve kinematics, odometry fused with vision, motor controller boilerplate, logging, and a way to organize subsystem behavior into states. NinjasLib packages all of that into independent, well-scoped modules so a team can:

- Pull in only the modules a given robot needs.
- Keep constants and hardware-specific code isolated from the core logic.
- Reuse the exact same drivetrain, vision, and logging code across seasons and robots, only editing constants files.
- Simulate and log everything through AdvantageKit, since most modules follow an `IO` interface + logged inputs pattern (real / sim / replay implementations behind a common interface).

## Features / Modules

The library is split into folders, where each folder is a self-contained module. Most hardware-facing modules follow the same **IO pattern**: an interface (e.g. `SwerveModuleIO`, `GyroIO`, `VisionCameraIO`) defines the contract, and separate `...IOReal`, `...IOSim` (and sometimes `...IOPigeon2`, `...IONavX`, etc.) classes implement it for different hardware or simulation. This keeps subsystem logic hardware-agnostic and simulation-friendly, and plugs directly into AdvantageKit-style logging.

### `swerve/`
Full swerve drivetrain implementation.
- `Swerve.java` – the drivetrain subsystem: kinematics, driving, and pose updates.
- `SwerveController.java` – closed-loop driving/heading control on top of the base swerve.
- `SwerveSpeeds.java`, `SwerveUtils.java` – speed containers and math/utility helpers.
- `constants/` – `SwerveConstants`, `SwerveControllerConstants`, `SwerveModuleConstants`: all tunable/robot-specific values for the drivetrain.
- `gyro/` – `Gyro` wrapper with `GyroIO`, and real implementations for `Pigeon2` and `NavX`, plus a `GyroIOSim`.
- `module/` – `SwerveModuleIO` with `SwerveModuleIOReal` and `SwerveModuleIOSim` implementations for an individual swerve module.

### `controllers/`
Thin, unified wrappers around common FRC motor controllers so subsystem code doesn't care which motor/controller it's driving.
- `Controller.java` – base abstraction.
- `SparkMaxController.java`, `TalonFXController.java`, `TalonSRXController.java`, `VictorSPXController.java` – concrete wrappers for REV and CTRE hardware.
- `SimulatedController.java` – a simulation-only controller implementation.
- `constants/` – `ControlConstants`, `ControllerConstants`, `RealControllerConstants`: PID/feedforward and hardware configuration for controllers.

### `localization/`
Pose estimation and vision fusion.
- `NinjasPoseTracker.java` / `NinjasSwervePoseTracker.java` – odometry-based pose tracking (the swerve variant fuses swerve module positions).
- `OdometryThread.java` – higher-frequency odometry sampling off the main loop.
- `vision/` – a full vision subsystem: `Vision.java`, `VisionCameraIO` with `LimelightVisionCameraIO`, `PhotonVisionCameraIO`, and `PhotonVisionSimCameraIO` implementations, plus `VisionOutput`, `VisionConstants`, `FieldLayoutGetter`, and the `LimelightHelpers` utility class for MegaTag pose fusion into the drivetrain's pose estimator.

### `statemachine/`
A lightweight state machine base for organizing robot/subsystem behavior.
- `RobotStateBase.java` – base for defining a robot's discrete states.
- `StateMachineBase.java` – base for creating a statemachine subsystem.

### `subsystem/`
Common subsystem contracts.
- `ISubsystem.java` – a shared interface subsystems can implement for consistent behavior (state handling, periodic hooks, etc.).
- `IO.java` – the generic IO interface pattern used throughout the library.

### `commands/`
Reusable, generic command patterns that aren't tied to a specific subsystem.
- `BackgroundCommand.java` – utility to help managing a command that is ran in the background.
- `DetachedCommand.java` – a command that starts running another command, and by that detaching it from the current command sequence.
- `LoopCommand.java` – a for-loop command. Runs a command N times.
- `StateEndCommand.java` – a command that defines an end-condition for statemachine state. It's a command sequence which when finishes, an end condition triggers.

### `loggedcontroller/`
Logs driver/operator controller (gamepad) input through AdvantageKit.
- `LoggedCommandController.java` with `LoggedCommandControllerIO` and a PS5-controller implementation (`LoggedCommandControllerIOPS5`), so controller input is replayable like any other subsystem input.

### `loggeddigitalinput/`
A logged wrapper around `DigitalInput` (e.g. limit switches, beam breaks).
- `LoggedDigitalInput.java` with `LoggedDigitalInputIO` and `Real`/`Sim` implementations, supporting inversion and an explicit disabled value.

### Root-level utilities
- `DerivativeCalculator.java` / `DerivativeCalculator2d.java` – numerical derivative helpers (1D and 2D) for computing velocity/acceleration from position samples.
- `LoggedTunableNumber.java` – a number that can be tuned live from the dashboard (e.g. NetworkTables/AdvantageScope) and is also logged, useful for on-the-fly PID/feedforward tuning.

## Code organization — where to look for what

The repo is organized by **module folders**, and inside modules with hardware variants, by an **interface + constants** pattern. Use this as a map when searching the codebase:

```
NinjasLib/
├── commands/                 → generic, reusable Command classes
├── controllers/               → motor controller wrappers (SparkMax, TalonFX, TalonSRX, VictorSPX, Sim)
│   └── constants/             → controller & control-loop constants
├── localization/               → pose estimation
│   └── vision/                → vision subsystem, camera IO (Limelight/PhotonVision), vision constants
├── loggedcontroller/          → logged gamepad/controller input (with IO variants)
├── loggeddigitalinput/        → logged DigitalInput wrapper (with IO variants)
├── statemachine/              → base classes for robot/subsystem state machines
├── subsystem/                 → shared subsystem interfaces (ISubsystem, IO)
├── swerve/                    → the swerve drivetrain
│   ├── constants/             → all swerve tuning/robot-specific constants
│   ├── gyro/                  → gyro abstraction + Pigeon2/NavX/Sim implementations
│   └── module/                → individual swerve module IO (Real/Sim)
├── DerivativeCalculator.java   → numeric derivative (1D)
├── DerivativeCalculator2d.java → numeric derivative (2D)
└── LoggedTunableNumber.java    → live-tunable, logged number
```

General rules of thumb when searching for something:
- **Looking for hardware-specific code?** Look inside a module's root file (e.g. `Swerve.java`, `Vision.java`) for the logic, and its `IO` interface + `...IOReal`/`...IOSim` files for the hardware/simulation implementation.
- **Looking for a constant/tunable value?** Check the module's `constants/` subfolder — constants are always kept separate from logic.
- **Looking for how state/behavior is organized?** Start at `statemachine/` and `subsystem/ISubsystem.java`.
- **Looking for how driver input reaches the robot?** Start at `loggedcontroller/`.

## How to use / implement in a robot project

NinjasLib is intended to be pulled into your robot project as source, using **git submodules**, so it lives under your project's `src/main/java/frc/lib/NinjasLib` and matches its `frc.lib.NinjasLib` package structure exactly.

### 1. Add NinjasLib as a submodule

From the root of your robot code repository:

```bash
git submodule add https://github.com/Ninjas4744-Organization/NinjasLib.git src/main/java/frc/lib/NinjasLib
git submodule update --init --recursive
```

If you need a specific season's branch (e.g. `2026`):

```bash
cd src/main/java/frc/lib/NinjasLib
git checkout 2026
cd ../../../../../..
git add src/main/java/frc/lib/NinjasLib
git commit -m "Add NinjasLib as a submodule"
```

### 2. Cloning a robot project that already includes it

Anyone cloning your robot repo afterward needs to pull the submodule too:

```bash
git clone --recurse-submodules <your-robot-repo-url>
```

Or, if already cloned without it:

```bash
git submodule update --init --recursive
```

### 3. Keeping NinjasLib up to date

To pull the latest changes from NinjasLib into your project:

```bash
cd src/main/java/frc/lib/NinjasLib
git pull origin 2026
cd ../../../../../..
git add src/main/java/frc/lib/NinjasLib
git commit -m "Update NinjasLib"
```

### 4. Dependencies

NinjasLib code relies on the vendor libraries used inside it (WPILib, and depending on which modules you use: CTRE Phoenix 6, REVLib, PhotonLib, and AdvantageKit/`littletonrobotics`). Make sure the corresponding vendor dependency JSONs are installed in your project through WPILib's VS Code extension (**WPILib: Manage Vendor Libraries → Install new libraries (online)**) before building, matching whichever controller/vision modules you actually use.

### 5. Using a module in your robot code

Once the submodule is in place, import the module(s) you need like any other class in your project, since it shares your project's package root:

```java
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
```

Typical setup pattern:

1. Create your own `constants` class (or fill in NinjasLib's constants classes, e.g. `SwerveConstants`, `SwerveModuleConstants`, `VisionConstants`) with your robot's physical/tuning values.
2. Instantiate the module's real/sim `IO` implementation based on `Robot.isReal()`, and pass it into the module's main class (e.g. `new Swerve(isReal ? new SwerveModuleIOReal(...) : new SwerveModuleIOSim(...), ...)`).
3. Register the module as a subsystem (implementing `ISubsystem` where applicable) and call its `periodic()` from your subsystem's periodic loop.
4. Wire up commands from `commands/` and controller input from `loggedcontroller/` in `RobotContainer`.
5. If using a state machine, extend `RobotStateBase` with your robot's states and drive it through `StateMachineBase`.

### 6. Removing the submodule

If you ever need to remove NinjasLib from a project:

```bash
git submodule deinit -f src/main/java/frc/lib/NinjasLib
git rm -f src/main/java/frc/lib/NinjasLib
rm -rf .git/modules/src/main/java/frc/lib/NinjasLib
```

---

*Maintained by Ninjas #4744 (Amal Hadera High School).*
