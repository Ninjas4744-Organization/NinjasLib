package frc.lib.NinjasLib.swerve.constants;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;

public class SwerveConstants {
    public Chassis chassis = new Chassis();
    public Limits limits = new Limits();
    public Modules modules = new Modules();
    public Gyro gyro = new Gyro();
    public Simulation simulation = new Simulation();
    public Special special = new Special();

    public static class Chassis {
        /** Distance between modules in forward axis */
        public double trackWidth;

        /** Distance between modules in side axis */
        public double wheelBase;

        /** Swerve kinematics class used for calculating swerve movement */
        public SwerveDriveKinematics kinematics;

        /** Width of bumper from side to side of robot, meters */
        public double bumperWidth;

        /** Length of bumper from back to forward of robot, meters */
        public double bumperLength;
    }

    public static class Limits {
        /** Max speed the swerve could possibly drive, the real thing if you want a speed limit do it in the speedLimit, in m/s */
        public double maxSpeed;

        /** Max speed the swerve could possibly rotate, the real thing if you want a speed limit do it in the rotationSpeedLimit, in rad/s */
        public double maxAngularVelocity;

        /** Max acceleration the swerve could possibly change movement direction (needs to be calibrated), in m/s^2 */
        public double maxSkidAcceleration;

        /** Swerve speed limit, the swerve can't drive faster than this number +-, in m/s */
        public double speedLimit;

        /** Swerve rotation speed limit, the swerve can't rotate faster than this number +-, in rad/s */
        public double rotationSpeedLimit;

        /** Swerve max acceleration limit, m/s^2 */
        public double accelerationLimit;

        /** Swerve max rotational acceleration limit, rad/s^2 */
        public double rotationAccelerationLimit;
    }

    public static class Modules {
        /** Whether to drive without module velocity PID control */
        public boolean openLoop;

        /** Module Specific Constants */
        public SwerveModuleConstants[] moduleConstants;

        /** Drive motor constants (Put whatever as ID and inverted and then put the real in the ModuleConstants) */
        public ControllerConstants driveMotorConstants;

        /** Steer motor constants (Put whatever as ID and inverted and then put the real in the ModuleConstants) */
        public ControllerConstants steerMotorConstants;

        /** The type of the controller of the drive motor */
        public Controller.ControllerType driveControllerType;

        /** The type of the controller of the steer motor */
        public Controller.ControllerType steerControllerType;
    }

    public static class Gyro {
        public enum GyroType {
            NavX,
            Pigeon2
        }

        /** Whether the gyro is navx or pigeon2 */
        public GyroType gyroType;

        /** ID of robot in CAN bus. Only if using pigeon2 and not navx */
        public int gyroID;

        /** Whether to invert the input from the gyro */
        public boolean gyroInverted;
    }

    public static class Simulation {
        /** The type of the drive motors */
        public DCMotor driveMotorType;

        /** The type of the drive motors */
        public DCMotor steerMotorType;
    }

    public static class Special {
        /**
         * Whether to create a separate thread to run the swerve odometry
         */
        public boolean enableOdometryThread;

        /**
         * Frequency of the thread that updates the odometry. Only works if enableOdometryThread is set to true
         */
        public int odometryThreadFrequency;

        /** Whether the robot is running in AdvantageKit replay simulation mode. */
        public boolean isReplay;

        /** Robot config */
        public RobotConfig robotConfig;

        /** The starting position of the robot */
        public Pose2d robotStartPose;

        /** The name of the canbus the swerve is running on. 'rio' by default if CANivore is not present */
        public CANBus CANBus = com.ctre.phoenix6.CANBus.roboRIO();
    }
}
