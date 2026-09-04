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
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;
import frc.lib.NinjasLib.localization.RobotPose;
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

/**
 * Manages a 4-module swerve drivetrain: hardware/simulation setup, acceleration- and speed-limited
 * driving via {@link #drive}, odometry feeding, and module/gyro access. Accessed as a singleton
 * through {@link #get()}/{@link #setInstance}; most callers should go through
 * {@link SwerveController} rather than this class directly.
 */
public class Swerve {
    private SwerveModuleIO[] modules;
    private SwerveDriveKinematics kinematics;
    private Gyro gyro;

    private SwerveSpeeds wantedSpeeds = new SwerveSpeeds();
    private SwerveModuleIOInputs[] moduleInputs;
    private SwerveModulePosition[] previousModulePositions;
    /** Guards module/gyro sampling against concurrent access between the main loop and the odometry thread. */
    public static final Lock odometryLock = new ReentrantLock();

    private SlewRateLimiter rotAccelerationLimit;
    private double maxSkidAcceleration;
    private double maxForwardAcceleration;

    private SwerveConstants constants;
    private SwerveDriveSimulation simulation;
    private static Swerve instance;
    private boolean disabled = false;

    /**
     * @return the singleton {@link Swerve} instance set via {@link #setInstance}, or a disabled
     *         no-op instance (logging a warning) if none has been set yet
     */
    public static Swerve get() {
        if (instance == null) {
            NinjasLogger.logEventImportant("Swerve not set. Initialize Swerve by setInstance.");
            return new Swerve(); // Disabled swerve
        }
        return instance;
    }

    /** Sets the singleton instance returned by {@link #get()}. Call once during robot init. */
    public static void setInstance(Swerve swerve) {
        instance = swerve;
    }

    private Swerve() {
        disabled = true;
    }

    /**
     * Builds and fully initializes the swerve drivetrain: creates the four modules and gyro
     * (real hardware or a MapleSim simulation, depending on {@link Robot#isReal()}), starts the
     * odometry thread if configured, and resets modules to their absolute encoders.
     *
     * @param constants the drivetrain's physical, module, gyro and behavior configuration
     */
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
     * The primary entry point for commanding the drivetrain: this is the method every driver
     * control loop and autonomous routine should call each periodic cycle to move the robot.
     * Drives the swerve towards the given speeds, subject to acceleration and speed limiting, and
     * commands the resulting module states.
     * <p>
     * The requested {@code input} is not applied directly; it is treated as a target that
     * {@link #wantedSpeeds} accelerates towards, bounded by the configured forward, skid and
     * rotational acceleration/speed limits. If auto-lock is enabled in
     * {@link SwerveConstants.Special#enableAutoLock} and the input stays within the jitter
     * prevention deadband for {@link SwerveConstants.Special#autoLockFrames} consecutive calls,
     * this method locks the wheels in an X pattern via {@link #lockWheelsToX()} and resets
     * {@link #wantedSpeeds} instead of driving. On a real robot the final chassis speeds are
     * discretized (see {@link ChassisSpeeds#discretize}) to compensate for the 20&nbsp;ms
     * command loop before being converted to module states. This method is a no-op if the swerve
     * was constructed disabled (see {@link #get()}).
     *
     * @param input the desired velocity, angular velocity, and whether it is field- or
     *              robot-relative
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

    /** Sets the max lateral (skid) acceleration, in m/s&sup2;, applied by {@link #drive}. */
    public void setMaxSkidAcceleration(double maxSkidAcceleration) {
        this.maxSkidAcceleration = maxSkidAcceleration;
    }

    /** Sets the max forward acceleration, in m/s&sup2;, applied by {@link #drive}. */
    public void setMaxForwardAcceleration(double maxForwardAcceleration) {
        this.maxForwardAcceleration = maxForwardAcceleration;
    }

    /**
     * Sets the rotational acceleration limit, in rad/s&sup2;, applied by {@link #drive}, rebuilding
     * the underlying {@link SlewRateLimiter} in place so it keeps its current output value.
     */
    public void setRotationAccelerationLimit(double rotationAccelerationLimit) {
        constants.speeds.rotationAccelerationLimit = rotationAccelerationLimit;

        double lastValue = rotAccelerationLimit.lastValue();
        rotAccelerationLimit = new SlewRateLimiter(rotationAccelerationLimit);
        rotAccelerationLimit.reset(lastValue);
    }

    /** Sets the max translational speed, in m/s, that {@link #drive} will command. */
    public void setSpeedLimit(double speedLimit) {
        constants.speeds.speedLimit = speedLimit;
    }

    /** Sets the max rotational speed, in rad/s, that {@link #drive} will command. */
    public void setRotationSpeedLimit(double rotationSpeedLimit) {
        constants.speeds.rotationSpeedLimit = rotationSpeedLimit;
    }

    /**
     * Must be called once per robot loop cycle (e.g. from {@code Robot.robotPeriodic()} or a
     * subsystem's {@code periodic()}). Applies any pending initial pose, refreshes gyro/module
     * inputs and feeds odometry (directly, or by draining the odometry thread's buffered samples
     * if {@link SwerveConstants.Special#enableOdometryThread} is set), and logs current/wanted
     * velocities. A no-op if this swerve was constructed disabled.
     */
    public void periodic() {
        if (disabled)
            return;

        if(constants.special.robotStartPose.getX() != -999) {
            RobotPose.get().setRobotPose(constants.special.robotStartPose);
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
                RobotPose.get().addOdometryUpdate(getModulePositions(), gyro.getYawOffsetted());
            else
                RobotPose.get().setRobotPose(simulation.getSimulatedDriveTrainPose());
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

                RobotPose.get().addTimedOdometryUpdate(odometryUpdateModulePositions, gyroYawArray[i], sampleTimestamps[i]);

                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Module Positions", odometryUpdateModulePositions);
                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Gyro Yaw", gyroYawArray[i]);
                NinjasLogger.log("Swerve/Odometry Thread/Samples/" + i + "/Timestamp", sampleTimestamps[i]);
            }
        } else
            RobotPose.get().setRobotPose(simulation.getSimulatedDriveTrainPose());

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
