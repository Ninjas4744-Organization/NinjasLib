package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIO;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOInputsAutoLogged;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOReal;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOSim;
import frc.robot.Robot;
import frc.robot.RobotState;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

public class Swerve {
    private final SwerveModuleIO[] modules;
    private final SwerveDriveKinematics kinematics;

    private ChassisSpeeds wantedRobotRelativeSpeeds = new ChassisSpeeds();
    private SwerveModuleIOInputsAutoLogged[] moduleInputs;
    private SwerveModulePosition[] previousModulePositions;

    private SlewRateLimiter xAccelerationLimit;
    private SlewRateLimiter yAccelerationLimit;
    private SlewRateLimiter rotAccelerationLimit;

    private final SwerveConstants constants;
    public static final Lock odometryLock = new ReentrantLock();
    private SwerveDriveSimulation simulation;
    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null)
            throw new RuntimeException("Swerve not set. Initialize Swerve by setInstance.");
        return instance;
    }

    public static void setInstance(Swerve swerve) {
        instance = swerve;
    }

    public Swerve(SwerveConstants constants) {
        this.constants = constants;

        xAccelerationLimit = new SlewRateLimiter(constants.accelerationLimit);
        yAccelerationLimit = new SlewRateLimiter(constants.accelerationLimit);
        rotAccelerationLimit = new SlewRateLimiter(constants.rotationAccelerationLimit);

        kinematics = constants.kinematics;

        moduleInputs = new SwerveModuleIOInputsAutoLogged[]{
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged()
        };

        if (Robot.isReal()) {
            modules = new SwerveModuleIO[]{
                new SwerveModuleIOReal(constants.moduleConstants[0], constants),
                new SwerveModuleIOReal(constants.moduleConstants[1], constants),
                new SwerveModuleIOReal(constants.moduleConstants[2], constants),
                new SwerveModuleIOReal(constants.moduleConstants[3], constants)
            };
        } else {
            DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(Kilograms.of(constants.robotConfig.massKG),
                Meters.of(constants.bumperLength), Meters.of(constants.bumperWidth),
                Meters.of(constants.trackWidth), Meters.of(constants.wheelBase),
                COTS.ofPigeon2(),
                COTS.ofMark4n(constants.driveMotorType, constants.steerMotorType, constants.robotConfig.moduleConfig.wheelCOF, 3));//() -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig));

            simulation = new SwerveDriveSimulation(config, new Pose2d(3, 3, Rotation2d.kZero));

            SimulatedArena.getInstance().addDriveTrainSimulation(simulation);

            modules = new SwerveModuleIO[]{
                new SwerveModuleIOSim(constants.moduleConstants[0], simulation.getModules()[0]),
                new SwerveModuleIOSim(constants.moduleConstants[1], simulation.getModules()[1]),
                new SwerveModuleIOSim(constants.moduleConstants[2], simulation.getModules()[2]),
                new SwerveModuleIOSim(constants.moduleConstants[3], simulation.getModules()[3])
            };
        }

        resetModulesToAbsolute();
    }

    /**
     * Drives the robot
     *
     * @param input The input to drive: speed, angular speed and field/robot relative
     */
    public void drive(SwerveInput input) {
//        Translation2d currentVelocity = new Translation2d(getChassisSpeeds(fieldRelative).vxMetersPerSecond, getChassisSpeeds(fieldRelative).vyMetersPerSecond);
        Translation2d currentVelocity = new SwerveInput(getChassisSpeeds(input.isFieldRelative()), input.isFieldRelative()).toTranslation();
//        Translation2d wantedVelocity = new Translation2d(drive.vxMetersPerSecond, drive.vyMetersPerSecond);
        Translation2d wantedVelocity = input.toTranslation();

        wantedVelocity = SwerveUtils.limitSkidAcceleration(currentVelocity, wantedVelocity, constants.maxSkidAcceleration);
//        ChassisSpeeds robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), drive.omegaRadiansPerSecond, RobotStateWithSwerve.getInstance().getGyroYaw()) : new ChassisSpeeds(wantedVelocity.getX(), wantedVelocity.getY(), drive.omegaRadiansPerSecond);
        ChassisSpeeds robotRelativeSpeeds = new SwerveInput(wantedVelocity.getX(), wantedVelocity.getY(), input.omegaRadiansPerSecond, input.isFieldRelative()).getAsRobotRelative(RobotState.getInstance().getGyroYaw());

        robotRelativeSpeeds = new ChassisSpeeds(
            xAccelerationLimit.calculate(MathUtil.clamp(robotRelativeSpeeds.vxMetersPerSecond, -constants.speedLimit, constants.speedLimit)),
            yAccelerationLimit.calculate(MathUtil.clamp(robotRelativeSpeeds.vyMetersPerSecond, -constants.speedLimit, constants.speedLimit)),
            rotAccelerationLimit.calculate(MathUtil.clamp(robotRelativeSpeeds.omegaRadiansPerSecond, -constants.rotationSpeedLimit, constants.rotationSpeedLimit))
        );

        wantedRobotRelativeSpeeds = robotRelativeSpeeds;
        setModuleStates(kinematics.toSwerveModuleStates(wantedRobotRelativeSpeeds), constants.openLoop);
    }

    public void stop() {
        drive(new SwerveInput());
    }

    public void setAccelerationLimit(double accelerationLimit) {
        constants.accelerationLimit = accelerationLimit;

        double lastValue = xAccelerationLimit.lastValue();
        xAccelerationLimit = new SlewRateLimiter(accelerationLimit);
        xAccelerationLimit.reset(lastValue);

        lastValue = yAccelerationLimit.lastValue();
        yAccelerationLimit = new SlewRateLimiter(accelerationLimit);
        yAccelerationLimit.reset(lastValue);
    }

    public void setRotationAccelerationLimit(double rotationAccelerationLimit) {
        constants.rotationAccelerationLimit = rotationAccelerationLimit;

        double lastValue = rotAccelerationLimit.lastValue();
        rotAccelerationLimit = new SlewRateLimiter(rotationAccelerationLimit);
        rotAccelerationLimit.reset(lastValue);
    }

    public void setSpeedLimit(double speedLimit) {
        constants.speedLimit = speedLimit;
    }

    public void setRotationSpeedLimit(double rotationSpeedLimit) {
        constants.rotationSpeedLimit = rotationSpeedLimit;
    }

    public void periodic() {
        Logger.recordOutput("Swerve/Current Velocity", getChassisSpeeds(true));
        Logger.recordOutput("Swerve/Wanted Velocity", ChassisSpeeds.fromRobotRelativeSpeeds(wantedRobotRelativeSpeeds, RobotStateWithSwerve.getInstance().getGyroYaw()));

        if (constants.enableOdometryThread) {
            odometryLock.lock();
            Rotation2d[] gyroYawArray = RobotStateWithSwerve.getInstance().getGyroYawArray();
            for (SwerveModuleIO module : modules) {
                module.periodic();

                module.updateInputs(moduleInputs[module.getModuleNumber()]);
                Logger.processInputs("Swerve/Module " + module.getModuleNumber(), moduleInputs[module.getModuleNumber()]);
            }
            odometryLock.unlock();

            if (Robot.isReal()) {
                double[] sampleTimestamps = moduleInputs[0].Timestamps;
                int sampleCount = sampleTimestamps.length;

                for (int i = 0; i < sampleCount; i++) {
                    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
                    for (int j = 0; j < 4; j++) {
                        int index = modules[i].getModuleNumber();
                        double drivePosition = moduleInputs[index].Positions[i];
                        Rotation2d steerAngle = moduleInputs[index].Angles[i];
                        modulePositions[index] = new SwerveModulePosition(drivePosition, steerAngle);
                    }

                    RobotStateWithSwerve.getInstance().updateRobotPoseWithTime(modulePositions, gyroYawArray[i], sampleTimestamps[i]);
                }
            } else
                RobotStateWithSwerve.getInstance().setRobotPose(simulation.getSimulatedDriveTrainPose());
        } else {
            if (Robot.isReal())
                RobotStateWithSwerve.getInstance().updateRobotPose(getModulePositions());
            else
                RobotStateWithSwerve.getInstance().setRobotPose(simulation.getSimulatedDriveTrainPose());
        }
    }

    public double getOdometryFrequency() {
        if (constants.enableOdometryThread)
            return constants.odometryThreadFrequency;
        return 50;
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.maxSpeed);
        for (SwerveModuleIO module : modules)
            module.setDesiredState(desiredStates[module.getModuleNumber()], isOpenLoop);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModuleIO module : modules)
            states[module.getModuleNumber()] = module.getState();
        return states;
    }

    public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        return fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(speeds, RobotStateWithSwerve.getInstance().getGyroYaw()) : speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModuleIO module : modules) positions[module.getModuleNumber()] = module.getPosition();
        return positions;
    }

    /** Resets the swerve modules to their absolute encoders */
    public void resetModulesToAbsolute() {
        if (Robot.isSimulation())
            return;

        System.out.println("---------------Resetting modules to absolute---------------");
        for (SwerveModuleIO module : modules)
            ((SwerveModuleIOReal) module).resetToAbsolute();
        System.out.println("---------------Resetting modules to absolute---------------");
    }

    public Rotation2d getGyroSimulationReading() {
        if (Robot.isSimulation())
            return simulation.getGyroSimulation().getGyroReading();
        return Rotation2d.kZero;
    }

    public Translation2d getOdometryTwist() {
        if (previousModulePositions == null) {
            previousModulePositions = getModulePositions();
            return new Translation2d();
        }
        Twist2d twist = kinematics.toTwist2d(previousModulePositions, getModulePositions());
        previousModulePositions = getModulePositions();
        return new Translation2d(twist.dx, twist.dy);
    }
}
