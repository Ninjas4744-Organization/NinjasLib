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
import frc.lib.NinjasLib.localization.OdometryThread;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.gyro.*;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIO;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOInputsAutoLogged;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOReal;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOSim;
import frc.robot.Robot;
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
    private final Gyro gyro;

    private SwerveSpeeds wantedSpeeds = new SwerveSpeeds();
    private SwerveModuleIOInputsAutoLogged[] moduleInputs;
    private SwerveModulePosition[] previousModulePositions;
    public static final Lock odometryLock = new ReentrantLock();

    private SlewRateLimiter rotAccelerationLimit;
    private double maxSkidAcceleration;
    private double maxForwardAcceleration;

    private final SwerveConstants constants;
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

        rotAccelerationLimit = new SlewRateLimiter(constants.limits.rotationAccelerationLimit);
        maxSkidAcceleration = constants.limits.maxSkidAcceleration;
        maxForwardAcceleration = constants.limits.maxForwardAcceleration;

        kinematics = constants.chassis.kinematics;

        moduleInputs = new SwerveModuleIOInputsAutoLogged[]{
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged(),
            new SwerveModuleIOInputsAutoLogged()
        };

        if (Robot.isReal()) {
            modules = new SwerveModuleIO[]{
                new SwerveModuleIOReal(constants.modules.moduleConstants[0], constants),
                new SwerveModuleIOReal(constants.modules.moduleConstants[1], constants),
                new SwerveModuleIOReal(constants.modules.moduleConstants[2], constants),
                new SwerveModuleIOReal(constants.modules.moduleConstants[3], constants)
            };

            if(constants.gyro.gyroType == SwerveConstants.Gyro.GyroType.NavX)
                gyro = new Gyro(new GyroIONavX(constants.special.odometryThreadFrequency, constants.gyro.gyroInverted));
            else
                gyro = new Gyro(new GyroIOPigeon2(constants.gyro.gyroID, constants.gyro.gyroInverted, constants.special.odometryThreadFrequency, constants.special.CANBus));

        } else if (!constants.special.isReplay) {
            DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(Kilograms.of(constants.special.robotConfig.massKG),
                Meters.of(constants.chassis.bumperLength), Meters.of(constants.chassis.bumperWidth),
                Meters.of(constants.chassis.trackWidth), Meters.of(constants.chassis.wheelBase),
                COTS.ofPigeon2(),
                COTS.ofMark4n(constants.simulation.driveMotorType, constants.simulation.steerMotorType, constants.special.robotConfig.moduleConfig.wheelCOF, 3));//() -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig), () -> new SwerveModuleSimulation(moduleConfig));

            simulation = new SwerveDriveSimulation(config, constants.special.robotStartPose);

            SimulatedArena.getInstance().addDriveTrainSimulation(simulation);

            modules = new SwerveModuleIO[]{
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[0], simulation.getModules()[0]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[1], simulation.getModules()[1]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[2], simulation.getModules()[2]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[3], simulation.getModules()[3])
            };

            gyro = new Gyro(new GyroIOSim(simulation.getGyroSimulation(), constants.gyro.gyroInverted));
        } else {
            modules = new SwerveModuleIO[]{
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {}
            };

            gyro = new Gyro(new GyroIO() {});
        }

        if(constants.special.enableOdometryThread)
            OdometryThread.getInstance().start(constants.special.odometryThreadFrequency);

        resetModulesToAbsolute();
    }

    /**
     * Drives the robot
     *
     * @param input The input to drive: speed, angular speed and field/robot relative
     */
    public void drive(SwerveSpeeds input) {
        wantedSpeeds = new SwerveSpeeds(
            SwerveUtils.limitForwardAndSkidAcceleration(
                wantedSpeeds.getAs(input.fieldRelative, gyro.getYaw()).toTranslation(),
                input.toTranslation(),
                maxForwardAcceleration,
                maxSkidAcceleration,
                constants.limits.maxSpeed),
            input.omegaRadiansPerSecond,
            input.fieldRelative);

        Translation2d clampedVel = wantedSpeeds.toTranslation();
        if (wantedSpeeds.getSpeed() > constants.limits.speedLimit)
            clampedVel = new Translation2d(constants.limits.speedLimit, clampedVel.getAngle());

        wantedSpeeds = new SwerveSpeeds(clampedVel,
            rotAccelerationLimit.calculate(MathUtil.clamp(wantedSpeeds.omegaRadiansPerSecond, -constants.limits.rotationSpeedLimit, constants.limits.rotationSpeedLimit)),
            wantedSpeeds.fieldRelative);
        wantedSpeeds = wantedSpeeds.getAsRobotRelative(gyro.getYaw());
        wantedSpeeds = new SwerveSpeeds(ChassisSpeeds.discretize(wantedSpeeds, 0.02), wantedSpeeds.fieldRelative);

        setModuleStates(kinematics.toSwerveModuleStates(wantedSpeeds), constants.modules.openLoop, true);
    }

    public void stop() {
        drive(new SwerveSpeeds());
    }

    public void lockWheelsToX() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
        }, constants.modules.openLoop, false);

//        return Commands.sequence(
//            Commands.runOnce(() -> {
//                setModuleStates(new SwerveModuleState[] {
//                    new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
//                    new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
//                    new SwerveModuleState(0.3, Rotation2d.fromDegrees(-45)),
//                    new SwerveModuleState(0.3, Rotation2d.fromDegrees(45)),
//                }, constants.modules.openLoop);
//            }),
//            Commands.waitUntil(() ->
//                       Math.abs(moduleInputs[0].Position.angle.minus(Rotation2d.fromDegrees(45)) .getCos()) > Math.cos(Units.degreesToRadians(5))
//                    && Math.abs(moduleInputs[1].Position.angle.minus(Rotation2d.fromDegrees(-45)).getCos()) > Math.cos(Units.degreesToRadians(5))
//                    && Math.abs(moduleInputs[2].Position.angle.minus(Rotation2d.fromDegrees(-45)).getCos()) > Math.cos(Units.degreesToRadians(5))
//                    && Math.abs(moduleInputs[3].Position.angle.minus(Rotation2d.fromDegrees(45)) .getCos()) > Math.cos(Units.degreesToRadians(5))),
//            Commands.runOnce(() -> {
//                setModuleStates(new SwerveModuleState[] {
//                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
//                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
//                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
//                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
//                }, constants.modules.openLoop);
//            })
//        );
    }

    public void setMaxSkidAcceleration(double maxSkidAcceleration) {
        this.maxSkidAcceleration = maxSkidAcceleration;
    }

    public void setMaxForwardAcceleration(double maxForwardAcceleration) {
        this.maxForwardAcceleration = maxForwardAcceleration;
    }

    public void setRotationAccelerationLimit(double rotationAccelerationLimit) {
        constants.limits.rotationAccelerationLimit = rotationAccelerationLimit;

        double lastValue = rotAccelerationLimit.lastValue();
        rotAccelerationLimit = new SlewRateLimiter(rotationAccelerationLimit);
        rotAccelerationLimit.reset(lastValue);
    }

    public void setSpeedLimit(double speedLimit) {
        constants.limits.speedLimit = speedLimit;
    }

    public void setRotationSpeedLimit(double rotationSpeedLimit) {
        constants.limits.rotationSpeedLimit = rotationSpeedLimit;
    }

    public void periodic() {
        if(constants.special.robotStartPose.getX() != -999){
            RobotStateWithSwerve.getInstance().setRobotPose(constants.special.robotStartPose);
            constants.special.robotStartPose = new Pose2d(-999, -999, Rotation2d.kZero);
        }

        if (constants.special.enableOdometryThread) {
            updateOdometryThread();
        } else {
            gyro.periodic();

            for (int i = 0; i < modules.length; i++) {
                modules[i].periodic();
                modules[i].updateInputs(moduleInputs[i]);
                Logger.processInputs("Swerve/Module " + moduleInputs[i].ModuleNumber, moduleInputs[i]);
            }

            if (Robot.isReal() || constants.special.isReplay)
                RobotStateWithSwerve.getInstance().updateRobotPose(getModulePositions(), gyro.getYawOffsetted());
            else
                RobotStateWithSwerve.getInstance().setRobotPose(simulation.getSimulatedDriveTrainPose());
        }

        Logger.recordOutput("Swerve/Current Velocity", getSpeeds().getAsFieldRelative());
        Logger.recordOutput("Swerve/Wanted Velocity", wantedSpeeds.getAsFieldRelative());
    }
    
    int frames = 0;
    int framesWithUpdate = 0;
    public void updateOdometryThread() {
        frames++;
        if (!odometryLock.tryLock())
            return;
        framesWithUpdate++;

        gyro.periodic();
        Rotation2d[] gyroYawArray = gyro.getOdometryYawPositions();
        for (int i = 0; i < modules.length; i++) {
            modules[i].periodic();
            modules[i].updateInputs(moduleInputs[i]);
            Logger.processInputs("Swerve/Module " + moduleInputs[i].ModuleNumber, moduleInputs[i]);
        }
        odometryLock.unlock();

        if (Robot.isReal()) {
            double[] sampleTimestamps = moduleInputs[0].Timestamps;
            int sampleCount = Math.min(Math.min(gyroYawArray.length, sampleTimestamps.length), moduleInputs[0].Positions.length);
            Logger.recordOutput("Swerve/Odometry Thread/Sample Count", sampleCount);

            for (int i = 0; i < sampleCount; i++) {
                SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
                for (int j = 0; j < 4; j++) {
                    double drivePosition = moduleInputs[j].Positions[i];
                    Rotation2d steerAngle = moduleInputs[j].Angles[i];
                    modulePositions[j] = new SwerveModulePosition(drivePosition, steerAngle);
                }

                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Module 0 Position", modulePositions[0]);
                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Module 1 Position", modulePositions[1]);
                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Module 2 Position", modulePositions[2]);
                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Module 3 Position", modulePositions[3]);
                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Gyro Yaw", gyroYawArray[i]);
                Logger.recordOutput("Swerve/Odometry Thread/Sample " + i + "/Timestamp", sampleTimestamps[i]);
                RobotStateWithSwerve.getInstance().updateRobotPoseWithTime(modulePositions, gyroYawArray[i], sampleTimestamps[i]);
            }
        } else
            RobotStateWithSwerve.getInstance().setRobotPose(simulation.getSimulatedDriveTrainPose());

        Logger.recordOutput("Swerve/Odometry Thread/Odometry Update Frames Percent", framesWithUpdate / (double)frames * 100);
    }

    public double getOdometryFrequency() {
        if (constants.special.enableOdometryThread)
            return constants.special.odometryThreadFrequency;
        return 50;
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean preventJittering) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.limits.maxSpeed);

        for (int i = 0; i < modules.length; i++)
            modules[i].setDesiredState(desiredStates[i], isOpenLoop, preventJittering);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            states[i] = moduleInputs[i].State;
        return states;
    }

    public SwerveSpeeds getSpeeds() {
        return new SwerveSpeeds(kinematics.toChassisSpeeds(getModuleStates()), false);
    }

    public SwerveSpeeds getWantedSpeeds() {
        return wantedSpeeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++)
            positions[i] = moduleInputs[i].Position;
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

    public Translation2d getOdometryTwist() {
        if (previousModulePositions == null) {
            previousModulePositions = getModulePositions();
            return new Translation2d();
        }
        Twist2d twist = kinematics.toTwist2d(previousModulePositions, getModulePositions());
        previousModulePositions = getModulePositions();
        return new Translation2d(twist.dx, twist.dy);
    }

    public Gyro getGyro() {
        return gyro;
    }
}
