package frc.lib.NinjasLib.swerve.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;

/**
 * Top-level, builder-style configuration for a swerve drivetrain: physical chassis dimensions, speed
 * and acceleration limits, module/motor configuration, gyro selection, simulation-only parameters, and
 * miscellaneous options, grouped into nested holder classes. Each {@code withX} method sets the
 * corresponding group and returns {@code this} for chaining.
 */
public class SwerveConstants {
    /** Physical chassis dimensions and kinematics. */
    public Chassis chassis = new Chassis();

    /** Speed, acceleration and rotation limits. */
    public Speeds speeds = new Speeds();

    /** Module and drive/steer motor configuration. */
    public Modules modules = new Modules();

    /** Gyro selection and configuration. */
    public Gyro gyro = new Gyro();

    /** Parameters only used when running in simulation. */
    public Simulation simulation = new Simulation();

    /** Miscellaneous/less common options (odometry thread, CAN bus, auto-lock, etc.). */
    public Special special = new Special();

    /** @param chassis physical chassis dimensions and kinematics to use */
    public SwerveConstants withChassis(Chassis chassis) {
        this.chassis = chassis;
        return this;
    }

    /** @param speeds speed, acceleration and rotation limits to use */
    public SwerveConstants withSpeeds(Speeds speeds) {
        this.speeds = speeds;
        return this;
    }

    /** @param modules module and drive/steer motor configuration to use */
    public SwerveConstants withModules(Modules modules) {
        this.modules = modules;
        return this;
    }

    /** @param gyro gyro selection and configuration to use */
    public SwerveConstants withGyro(Gyro gyro) {
        this.gyro = gyro;
        return this;
    }

    /** @param simulation simulation-only parameters to use */
    public SwerveConstants withSimulation(Simulation simulation) {
        this.simulation = simulation;
        return this;
    }

    /** @param special miscellaneous options to use */
    public SwerveConstants withSpecial(Special special) {
        this.special = special;
        return this;
    }

    /** Physical dimensions of the chassis and the kinematics derived from them. */
    public static class Chassis {
        /** Distance between modules in forward axis */
        public double trackWidth = 0.6;

        /** Distance between modules in side axis */
        public double wheelBase = 0.6;

        /** Swerve kinematics class used for calculating swerve movement */
        public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        /** Width of bumper from side to side of robot, meters */
        public double bumperWidth = 0.9;

        /** Length of bumper from back to forward of robot, meters */
        public double bumperLength = 0.9;

        /** Sets track width and wheel base together and recomputes kinematics from them, since kinematics is derived from both. */
        public Chassis withDimensions(double trackWidth, double wheelBase) {
            this.trackWidth = trackWidth;
            this.wheelBase = wheelBase;
            this.kinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            );
            return this;
        }

        /** @param bumperWidth width of the bumper, side to side, in meters
         *  @param bumperLength length of the bumper, back to front, in meters */
        public Chassis withBumper(double bumperWidth, double bumperLength) {
            this.bumperWidth = bumperWidth;
            this.bumperLength = bumperLength;
            return this;
        }
    }

    /** Physical speed/acceleration capabilities of the swerve and the soft limits enforced on them. */
    public static class Speeds {
        /** Max speed the swerve could possibly drive, the real thing if you want a speed limit do it in the speedLimit, in m/s */
        public double maxSpeed = 5;

        /** Max speed the swerve could possibly rotate, the real thing if you want a speed limit do it in the rotationSpeedLimit, in rad/s */
        public double maxAngularVelocity = 9;

        /** Max acceleration the swerve could possibly change movement direction (needs to be calibrated), in m/s^2 */
        public double maxSkidAcceleration = Double.POSITIVE_INFINITY;

        /** Max acceleration the swerve could possibly change speed from 0 m/s (needs to be calibrated), in m/s^2 */
        public double maxForwardAcceleration = Double.POSITIVE_INFINITY;

        /** Swerve speed limit, the swerve can't drive faster than this number +-, in m/s */
        public double speedLimit = Double.POSITIVE_INFINITY;

        /** Swerve rotation speed limit, the swerve can't rotate faster than this number +-, in rad/s */
        public double rotationSpeedLimit = Double.POSITIVE_INFINITY;

        /** Swerve max rotational acceleration limit, rad/s^2 */
        public double rotationAccelerationLimit = Double.POSITIVE_INFINITY;

        /** Multiplier for discretization calculation in swerve */
        public double discretizeFactor = 1;

        /** The true maximum speeds the swerve is physically capable of (not a driving limit). */
        public Speeds withMaxSpeeds(double maxSpeed, double maxAngularVelocity) {
            this.maxSpeed = maxSpeed;
            this.maxAngularVelocity = maxAngularVelocity;
            return this;
        }

        /** Calibrated acceleration limits used for skid/forward/rotation acceleration control. */
        public Speeds withAccelerationLimits(double maxSkidAcceleration, double maxForwardAcceleration, double rotationAccelerationLimit) {
            this.maxSkidAcceleration = maxSkidAcceleration;
            this.maxForwardAcceleration = maxForwardAcceleration;
            this.rotationAccelerationLimit = rotationAccelerationLimit;
            return this;
        }

        /** The soft driving limits actually enforced on the swerve during teleop/auto. */
        public Speeds withSpeedLimits(double speedLimit, double rotationSpeedLimit) {
            this.speedLimit = speedLimit;
            this.rotationSpeedLimit = rotationSpeedLimit;
            return this;
        }

        /** @param discretizeFactor multiplier applied when discretizing chassis speeds */
        public Speeds withDiscretizeFactor(double discretizeFactor) {
            this.discretizeFactor = discretizeFactor;
            return this;
        }
    }

    /** Configuration shared by all swerve modules: motor types/constants and per-module hardware IDs. */
    public static class Modules {
        /** Whether to drive without module velocity PID control */
        public boolean openLoop = false;

        /** Module Specific Constants */
        public SwerveModuleConstants[] moduleConstants;

        /** Drive motor constants (Put whatever as ID and inverted and then put the real in the ModuleConstants) */
        public ControllerConstants driveMotorConstants;

        /** Steer motor constants (Put whatever as ID and inverted and then put the real in the ModuleConstants) */
        public ControllerConstants steerMotorConstants;

        /** The type of the controller of the drive motor */
        public Controller.ControllerType driveControllerType = Controller.ControllerType.TalonFX;

        /** The type of the controller of the steer motor */
        public Controller.ControllerType steerControllerType = Controller.ControllerType.TalonFX;

        /** Fraction of max speed below which a module holds its last angle instead of rotating, to prevent jittering. */
        public double jitterPreventionPercent = 0.01;

        /** @param moduleConstants per-module hardware configuration, one entry per swerve module */
        public Modules withModuleConstants(SwerveModuleConstants[] moduleConstants) {
            this.moduleConstants = moduleConstants;
            return this;
        }

        /** @param openLoop whether to drive without module velocity PID control */
        public Modules withOpenLoop(boolean openLoop) {
            this.openLoop = openLoop;
            return this;
        }

        /** Drive motor's constants and controller type together, since they describe the same physical motor. */
        public Modules withDriveMotor(ControllerConstants driveMotorConstants, Controller.ControllerType driveControllerType) {
            this.driveMotorConstants = driveMotorConstants;
            this.driveControllerType = driveControllerType;
            return this;
        }

        /** Steer motor's constants and controller type together, since they describe the same physical motor. */
        public Modules withSteerMotor(ControllerConstants steerMotorConstants, Controller.ControllerType steerControllerType) {
            this.steerMotorConstants = steerMotorConstants;
            this.steerControllerType = steerControllerType;
            return this;
        }
    }

    /** Which gyro hardware to build (via {@code frc.lib.NinjasLib.swerve.gyro.Gyro}) and how to configure it. */
    public static class Gyro {
        /** Supported gyro hardware types. */
        public enum GyroType {
            /** A NavX (AHRS) IMU connected over SPI. */
            NavX,
            /** A CTRE Pigeon 2 IMU connected over CAN. */
            Pigeon2
        }

        /** Whether the gyro is navx or pigeon2 */
        public GyroType gyroType = GyroType.Pigeon2;

        /** ID of robot in CAN bus. Only if using pigeon2 and not navx */
        public int gyroID = 5;

        /** Whether to invert the input from the gyro */
        public boolean gyroInverted = false;

        /** Creates a default gyro configuration: Pigeon2, CAN ID 5, not inverted. */
        public Gyro() {}

        /**
         * @param gyroID CAN ID of the gyro (only used when {@code gyroType} is {@link GyroType#Pigeon2})
         * @param gyroInverted whether to invert the gyro's input
         * @param gyroType which gyro hardware to use
         */
        public Gyro(int gyroID, boolean gyroInverted, GyroType gyroType) {
            this.gyroID = gyroID;
            this.gyroInverted = gyroInverted;
            this.gyroType = gyroType;
        }
    }

    /** Parameters used only by the physics simulation (motor models and module gearbox identity); ignored on a real robot. */
    public static class Simulation {
        /** Supported swerve module gearbox families (WCP/SDS MK4 variants). */
        public enum SwerveType {
            /** MK4 module. */
            Mark4,
            /** MK4i module. */
            Mark4i,
            /** MK4n module. */
            Mark4n,
        }

        /** The type of the drive motors */
        public DCMotor driveMotorType = DCMotor.getKrakenX60Foc(1);

        /** The type of the steer motors */
        public DCMotor steerMotorType = DCMotor.getKrakenX60Foc(1);

        /** The gear ratio level- L1, L2, L3, L4... */
        public int gearRatioLevel = 2;

        /** The swerve type- MK4, MK4i, MK4n*/
        public SwerveType swerveType = SwerveType.Mark4n;

        /**
         * @param driveMotorType simulated motor model used for the drive motors
         * @param steerMotorType simulated motor model used for the steer motors
         */
        public Simulation withMotors(DCMotor driveMotorType, DCMotor steerMotorType) {
            this.driveMotorType = driveMotorType;
            this.steerMotorType = steerMotorType;
            return this;
        }

        /** Physical gearbox identity- type and gear ratio level go together. */
        public Simulation withSwerveType(SwerveType swerveType, int gearRatioLevel) {
            this.swerveType = swerveType;
            this.gearRatioLevel = gearRatioLevel;
            return this;
        }
    }

    /** Less commonly changed options: odometry threading, PathPlanner robot config, starting pose, CAN bus, and auto-lock. */
    public static class Special {
        /** Whether to create a separate thread to run the swerve odometry */
        public boolean enableOdometryThread = false;

        /** Frequency of the thread that updates the odometry. Only works if enableOdometryThread is set to true */
        public int odometryThreadFrequency = 50;

        /** Robot config */
        public RobotConfig robotConfig;

        /** The starting position of the robot */
        public Pose2d robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);

        /** The name of the canbus the swerve is running on. 'rio' by default if CANivore is not present */
        public CANBus CANBus = com.ctre.phoenix6.CANBus.roboRIO();

        /** Whether to automatically lock swerve wheels to X after certain amount of zero input frames */
        public boolean enableAutoLock = false;

        /** How many zero input frames needed to auto lock wheels to X (Only works if enableAutoLock = true) */
        public int autoLockFrames = 50;

        /** Odometry thread toggle and its frequency go together, since frequency is meaningless when disabled. */
        public Special withOdometryThread(int odometryThreadFrequency) {
            this.enableOdometryThread = true;
            this.odometryThreadFrequency = odometryThreadFrequency;
            return this;
        }

        /** @param robotConfig the PathPlanner robot configuration to use */
        public Special withRobotConfig(RobotConfig robotConfig) {
            this.robotConfig = robotConfig;
            return this;
        }

        /** @param robotStartPose the field-relative pose the robot starts (and odometry resets to) */
        public Special withRobotStartPose(Pose2d robotStartPose) {
            this.robotStartPose = robotStartPose;
            return this;
        }

        /** @param CANBus the CAN bus the swerve hardware is connected to */
        public Special withCANBus(CANBus CANBus) {
            this.CANBus = CANBus;
            return this;
        }

        /** Auto-lock toggle and its frame threshold go together, since the threshold is meaningless when disabled. */
        public Special withAutoLock(int autoLockFrames) {
            this.enableAutoLock = true;
            this.autoLockFrames = autoLockFrames;
            return this;
        }
    }
}