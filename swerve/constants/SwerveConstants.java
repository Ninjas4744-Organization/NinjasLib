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

public class SwerveConstants {
    public Chassis chassis = new Chassis();
    public Speeds speeds = new Speeds();
    public Modules modules = new Modules();
    public Gyro gyro = new Gyro();
    public Simulation simulation = new Simulation();
    public Special special = new Special();

    public SwerveConstants withChassis(Chassis chassis) {
        this.chassis = chassis;
        return this;
    }

    public SwerveConstants withSpeeds(Speeds speeds) {
        this.speeds = speeds;
        return this;
    }

    public SwerveConstants withModules(Modules modules) {
        this.modules = modules;
        return this;
    }

    public SwerveConstants withGyro(Gyro gyro) {
        this.gyro = gyro;
        return this;
    }

    public SwerveConstants withSimulation(Simulation simulation) {
        this.simulation = simulation;
        return this;
    }

    public SwerveConstants withSpecial(Special special) {
        this.special = special;
        return this;
    }

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

        public Chassis withBumper(double bumperWidth, double bumperLength) {
            this.bumperWidth = bumperWidth;
            this.bumperLength = bumperLength;
            return this;
        }
    }

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

        public Speeds withDiscretizeFactor(double discretizeFactor) {
            this.discretizeFactor = discretizeFactor;
            return this;
        }
    }

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

        public double jitterPreventionPercent = 0.01;

        public Modules withModuleConstants(SwerveModuleConstants[] moduleConstants) {
            this.moduleConstants = moduleConstants;
            return this;
        }

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

    public static class Gyro {
        public enum GyroType {
            NavX,
            Pigeon2
        }

        /** Whether the gyro is navx or pigeon2 */
        public GyroType gyroType = GyroType.Pigeon2;

        /** ID of robot in CAN bus. Only if using pigeon2 and not navx */
        public int gyroID = 5;

        /** Whether to invert the input from the gyro */
        public boolean gyroInverted = false;

        public Gyro() {}

        public Gyro(int gyroID, boolean gyroInverted, GyroType gyroType) {
            this.gyroID = gyroID;
            this.gyroInverted = gyroInverted;
            this.gyroType = gyroType;
        }
    }

    public static class Simulation {
        public enum SwerveType {
            Mark4,
            Mark4i,
            Mark4n,
        }

        /** The type of the drive motors */
        public DCMotor driveMotorType = DCMotor.getKrakenX60Foc(1);

        /** The type of the drive motors */
        public DCMotor steerMotorType = DCMotor.getKrakenX60Foc(1);

        /** The gear ratio level- L1, L2, L3, L4... */
        public int gearRatioLevel = 2;

        /** The swerve type- MK4, MK4i, MK4n*/
        public SwerveType swerveType = SwerveType.Mark4n;

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

        public Special withRobotConfig(RobotConfig robotConfig) {
            this.robotConfig = robotConfig;
            return this;
        }

        public Special withRobotStartPose(Pose2d robotStartPose) {
            this.robotStartPose = robotStartPose;
            return this;
        }

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