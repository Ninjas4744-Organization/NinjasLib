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
import frc.lib.NinjasLib.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.gyro.*;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIO;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIO.SwerveModuleIOInputs;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOReal;
import frc.lib.NinjasLib.swerve.module.SwerveModuleIOSim;
import frc.robot.Robot;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

public class Swerve {
    private SwerveModuleIO[] modules;
    private SwerveDriveKinematics kinematics;
    private Gyro gyro;

    private SwerveSpeeds wantedSpeeds = new SwerveSpeeds();
    private SwerveModuleIOInputs[] moduleInputs;
    private SwerveModulePosition[] previousModulePositions;
    public static final Lock odometryLock = new ReentrantLock();

    private SlewRateLimiter rotAccelerationLimit;
    private double maxSkidAcceleration;
    private double maxForwardAcceleration;

    private SwerveConstants constants;
    private SwerveDriveSimulation simulation;
    private static Swerve instance;
    private boolean disabled = false;

    public static Swerve getInstance() {
        if (instance == null) {
            NinjasLogger.logEventImportant("Swerve not set. Initialize Swerve by setInstance.");
            return new Swerve(); // Disabled swerve
        }
        return instance;
    }

    public static void setInstance(Swerve swerve) {
        instance = swerve;
    }

    private Swerve() {
        disabled = true;
    }

    public Swerve(SwerveConstants constants) {
        this.constants = constants;

        rotAccelerationLimit = new SlewRateLimiter(constants.speeds.rotationAccelerationLimit);
        maxSkidAcceleration = constants.speeds.maxSkidAcceleration;
        maxForwardAcceleration = constants.speeds.maxForwardAcceleration;

        kinematics = constants.chassis.kinematics;

        moduleInputs = new SwerveModuleIOInputs[] {
            new SwerveModuleIOInputs(),
            new SwerveModuleIOInputs(),
            new SwerveModuleIOInputs(),
            new SwerveModuleIOInputs()
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

        } else {
            DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(
                Kilograms.of(constants.special.robotConfig.massKG),
                Meters.of(constants.chassis.bumperLength), Meters.of(constants.chassis.bumperWidth),
                Meters.of(constants.chassis.trackWidth), Meters.of(constants.chassis.wheelBase),
                COTS.ofPigeon2(),
                switch (constants.simulation.swerveType) {
                    case Mark4 -> COTS.ofMark4(constants.simulation.driveMotorType, constants.simulation.steerMotorType, constants.special.robotConfig.moduleConfig.wheelCOF, constants.simulation.gearRatioLevel);
                    case Mark4i -> COTS.ofMark4i(constants.simulation.driveMotorType, constants.simulation.steerMotorType, constants.special.robotConfig.moduleConfig.wheelCOF, constants.simulation.gearRatioLevel);
                    default -> COTS.ofMark4n(constants.simulation.driveMotorType, constants.simulation.steerMotorType, constants.special.robotConfig.moduleConfig.wheelCOF, constants.simulation.gearRatioLevel);
                }
            );

            simulation = new SwerveDriveSimulation(config, constants.special.robotStartPose);

            SimulatedArena.getInstance().addDriveTrainSimulation(simulation);

            modules = new SwerveModuleIO[]{
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[0], simulation.getModules()[0]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[1], simulation.getModules()[1]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[2], simulation.getModules()[2]),
                new SwerveModuleIOSim(constants, constants.modules.moduleConstants[3], simulation.getModules()[3])
            };

            gyro = new Gyro(new GyroIOSim(simulation.getGyroSimulation(), constants.gyro.gyroInverted));
        }

        if(constants.special.enableOdometryThread)
            OdometryThread.getInstance().start(constants.special.odometryThreadFrequency);

        resetModulesToAbsolute();
    }

    private int amountOfZeroInputFrames = 0;
    /**
     * Drives the swerve. Applies limit calculations.
     * @param input The input to drive: velocity, angular velocity and field/robot relative.
     */
    public void drive(SwerveSpeeds input) {
        if (disabled)
            return;

        if (constants.special.enableAutoLock) {
            if (input.toTranslation().getNorm() < constants.modules.jitterPreventionPercent * constants.speeds.maxSpeed && Math.abs(input.omegaRadiansPerSecond) < constants.modules.jitterPreventionPercent * constants.speeds.maxAngularVelocity)
                amountOfZeroInputFrames++;
            else amountOfZeroInputFrames = 0;

            if (amountOfZeroInputFrames >= constants.special.autoLockFrames) {
                lockWheelsToX();
                wantedSpeeds = new SwerveSpeeds();
                return;
            }
        }

        wantedSpeeds = new SwerveSpeeds(
            SwerveUtils.limitForwardAndSkidAcceleration(
                wantedSpeeds.getAs(input.fieldRelative, gyro.getYaw()).toTranslation(),
                input.toTranslation(),
                maxForwardAcceleration,
                maxSkidAcceleration,
                constants.speeds.maxSpeed),
            input.omegaRadiansPerSecond,
            input.fieldRelative);

        Translation2d clampedVel = wantedSpeeds.toTranslation();
        if (wantedSpeeds.getSpeed() > constants.speeds.speedLimit)
            clampedVel = new Translation2d(constants.speeds.speedLimit, clampedVel.getAngle());

        wantedSpeeds = new SwerveSpeeds(clampedVel,
            rotAccelerationLimit.calculate(MathUtil.clamp(wantedSpeeds.omegaRadiansPerSecond, -constants.speeds.rotationSpeedLimit, constants.speeds.rotationSpeedLimit)),
            wantedSpeeds.fieldRelative);

        wantedSpeeds = wantedSpeeds.getAsRobotRelative(gyro.getYaw());
        if (Robot.isReal())
            wantedSpeeds = new SwerveSpeeds(ChassisSpeeds.discretize(wantedSpeeds, 0.02 * constants.speeds.discretizeFactor), wantedSpeeds.fieldRelative);

        setModuleStates(kinematics.toSwerveModuleStates(wantedSpeeds), constants.modules.openLoop, true);
    }

    /**
     * Stops swerve. Empty drive request.
     */
    public void stop() {
        drive(new SwerveSpeeds());
    }

    /**
     * Puts all wheels/modules in X orientation to stop robot hard.
     */
    public void lockWheelsToX() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        }, constants.modules.openLoop, false);
    }

    public void setMaxSkidAcceleration(double maxSkidAcceleration) {
        this.maxSkidAcceleration = maxSkidAcceleration;
    }

    public void setMaxForwardAcceleration(double maxForwardAcceleration) {
        this.maxForwardAcceleration = maxForwardAcceleration;
    }

    public void setRotationAccelerationLimit(double rotationAccelerationLimit) {
        constants.speeds.rotationAccelerationLimit = rotationAccelerationLimit;

        double lastValue = rotAccelerationLimit.lastValue();
        rotAccelerationLimit = new SlewRateLimiter(rotationAccelerationLimit);
        rotAccelerationLimit.reset(lastValue);
    }

    public void setSpeedLimit(double speedLimit) {
        constants.speeds.speedLimit = speedLimit;
    }

    public void setRotationSpeedLimit(double rotationSpeedLimit) {
        constants.speeds.rotationSpeedLimit = rotationSpeedLimit;
    }

    public void periodic() {
        if (disabled)
            return;

        if(constants.special.robotStartPose.getX() != -999) {
            RobotStateBase.get().setRobotPose(constants.special.robotStartPose);
            constants.special.robotStartPose = new Pose2d(-999, -999, Rotation2d.kZero);
        }

        if (constants.special.enableOdometryThread) {
            updateOdometryThread();
        } else {
            gyro.periodic();

            for (int i = 0; i < modules.length; i++) {
                modules[i].periodic();
                moduleInputs[i] = modules[i].update();
                NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/State", moduleInputs[i].state);
                NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Desired State", moduleInputs[i].desiredState);
                NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Position", moduleInputs[i].position);
                NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Absolute Position", moduleInputs[i].absolutePosition);
            }

            if (Robot.isReal())
                RobotStateBase.get().updateRobotPose(getModulePositions(), gyro.getYawOffsetted());
            else
                RobotStateBase.get().setRobotPose(simulation.getSimulatedDriveTrainPose());
        }

        NinjasLogger.log("Swerve/Current Velocity", getSpeeds().getAsFieldRelative());
        NinjasLogger.log("Swerve/Wanted Velocity", wantedSpeeds.getAsFieldRelative());
    }
    
    private int odometryUpdateFrames = 0;
    private int odometryUpdateFramesWithUpdate = 0;
    private final SwerveModulePosition[] odometryUpdateModulePositions = new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    private void updateOdometryThread() {
        odometryUpdateFrames++;
        if (!odometryLock.tryLock())
            return;
        odometryUpdateFramesWithUpdate++;

        gyro.periodic();
        Rotation2d[] gyroYawArray = gyro.getOdometryYawPositions();
        for (int i = 0; i < modules.length; i++) {
            modules[i].periodic();
            moduleInputs[i] = modules[i].update();

            NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/State", moduleInputs[i].state);
            NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Desired State", moduleInputs[i].desiredState);
            NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Position", moduleInputs[i].position);
            NinjasLogger.log("Swerve/Module " + moduleInputs[i].moduleNumber + "/Absolute Position", moduleInputs[i].absolutePosition);
        }
        odometryLock.unlock();

        if (Robot.isReal()) {
            double[] sampleTimestamps = moduleInputs[0].timestamps;
            int sampleCount = Math.min(Math.min(gyroYawArray.length, sampleTimestamps.length), moduleInputs[0].positions.length);
            sampleCount = Math.min(sampleCount, 5);
            NinjasLogger.log("Swerve/Odometry Thread/Sample Count", sampleCount);

            for (int i = 0; i < sampleCount; i++) {
                for (int j = 0; j < 4; j++) {
                    odometryUpdateModulePositions[j].distanceMeters = moduleInputs[j].positions[i];
                    odometryUpdateModulePositions[j].angle = moduleInputs[j].angles[i];
                }

                RobotStateBase.get().updateRobotPoseWithTime(odometryUpdateModulePositions, gyroYawArray[i], sampleTimestamps[i]);

                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Module Positions", odometryUpdateModulePositions);
                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Gyro Yaw", gyroYawArray[i]);
                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Timestamp", sampleTimestamps[i]);
            }
        } else
            RobotStateBase.get().setRobotPose(simulation.getSimulatedDriveTrainPose());

        NinjasLogger.log("Swerve/Odometry Thread/Odometry Update Frames Percent", odometryUpdateFramesWithUpdate / (double) odometryUpdateFrames * 100);
    }

    /**
     * @return How many times odometry updates in a second. 50 by default but can be different if using odometry thread.
     */
    public double getOdometryFrequency() {
        if (disabled)
            return 50;

        if (constants.special.enableOdometryThread)
            return constants.special.odometryThreadFrequency;
        return 50;
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean preventJittering) {
        if (disabled)
            return;

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.speeds.maxSpeed);

        for (int i = 0; i < modules.length; i++)
            modules[i].setDesiredState(desiredStates[i], isOpenLoop, preventJittering);
    }

    /**
     * @return State(Speed (m/s), Angle) of each swerve module
     */
    public SwerveModuleState[] getModuleStates() {
        if (disabled)
            return new SwerveModuleState[0];

        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            states[i] = moduleInputs[i].state;
        return states;
    }

    /**
     * @return Current speed of swerve according to odometry. m/s
     */
    public SwerveSpeeds getSpeeds() {
        if (disabled)
            return new SwerveSpeeds();

        return new SwerveSpeeds(kinematics.toChassisSpeeds(getModuleStates()), false);
    }

    /**
     * @return Wanted speeds of swerve. The requested swerve input after limits calculations. m/s
     */
    public SwerveSpeeds getWantedSpeeds() {
        return wantedSpeeds;
    }

    /**
     * @return Position(Distance (m), Angle) of each swerve module
     */
    public SwerveModulePosition[] getModulePositions() {
        if (disabled)
            return new  SwerveModulePosition[0];

        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++)
            positions[i] = moduleInputs[i].position;
        return positions;
    }

    /** Resets the swerve modules to their absolute encoders */
    public void resetModulesToAbsolute() {
        if (Robot.isSimulation() || disabled)
            return;

        NinjasLogger.logEvent("Resetting modules to absolute");
        for (SwerveModuleIO module : modules)
            ((SwerveModuleIOReal) module).resetToAbsolute();
    }

    /**
     * @return Odometry translation from last call of this function to this call
     */
    public Translation2d getOdometryTwist() {
        if (disabled)
            return new Translation2d();

        if (previousModulePositions == null) {
            previousModulePositions = getModulePositions();
            return new Translation2d();
        }
        Twist2d twist = kinematics.toTwist2d(previousModulePositions, getModulePositions());
        previousModulePositions = getModulePositions();
        return new Translation2d(twist.dx, twist.dy);
    }

    /**
     * @return Gyro object
     */
    public Gyro getGyro() {
        return gyro;
    }
}
