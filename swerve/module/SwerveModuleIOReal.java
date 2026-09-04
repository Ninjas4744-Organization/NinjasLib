package frc.lib.NinjasLib.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.TalonFXController;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.localization.OdometryThread;
import frc.lib.NinjasLib.swerve.SwerveUtils;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;

import java.util.Queue;

/**
 * {@link SwerveModuleIO} implementation for a real swerve module driven by {@link Controller}-based
 * drive/steer motors and a CTRE CANCoder absolute encoder.
 */
public class SwerveModuleIOReal implements SwerveModuleIO {
    /** Index of this module within the swerve drive. */
    public final int moduleNumber;
    private final SwerveConstants swerveConstants;

    private final Controller steerMotor;
    private final Controller driveMotor;

    private Rotation2d lastAngle;
    private final CANcoder canCoder;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final boolean isTalonFX;
    private Queue<Double> positionQueue;
    private Queue<Double> angleQueue;
    private Queue<Double> timestampQueue;

    private int absolutePositionUpdateCounter = 0;
    private static final int ABSOLUTE_POSITION_UPDATE_PERIOD = 10;
    private Rotation2d cachedAbsolutePosition = new Rotation2d();

    /**
     * Constructs the module's drive/steer controllers and CANCoder from the given constants, and
     * registers their position signals with the {@link OdometryThread} when
     * {@code swerveConstants.special.enableOdometryThread} is set and both motors are TalonFX-controlled.
     *
     * @param constants this module's specific IDs, inversions and CANCoder offset
     * @param swerveConstants the shared swerve constants (motor types, CAN bus, odometry settings, etc.)
     */
    public SwerveModuleIOReal(SwerveModuleConstants constants, SwerveConstants swerveConstants) {
        moduleNumber = constants.moduleNumber;
        this.swerveConstants = swerveConstants;

        ControllerConstants driveMotorConstants = swerveConstants.modules.driveMotorConstants.clone();
        driveMotorConstants.real.base.main.id = constants.driveMotorID;
        driveMotorConstants.real.base.main.inverted = constants.driveMotorInverted;

        ControllerConstants steerMotorConstants = swerveConstants.modules.steerMotorConstants.clone();
        steerMotorConstants.real.base.main.id = constants.steerMotorID;
        steerMotorConstants.real.base.main.inverted = constants.steerMotorInverted;

        if(swerveConstants.special.CANBus.getName().equals("rio"))
            canCoder = new CANcoder(constants.canCoderID);
        else {
            canCoder = new CANcoder(constants.canCoderID, swerveConstants.special.CANBus);
            driveMotorConstants.real.base.CANBus = swerveConstants.special.CANBus;
            steerMotorConstants.real.base.CANBus = swerveConstants.special.CANBus;
        }
        canCoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withSensorDirection(constants.invertCANCoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(constants.CANCoderOffset)
        );

        driveMotor = Controller.createController(swerveConstants.modules.driveControllerType, driveMotorConstants);
        steerMotor = Controller.createController(swerveConstants.modules.steerControllerType, steerMotorConstants);

        lastAngle = Rotation2d.fromRadians(steerMotor.getPosition());

        isTalonFX = swerveConstants.modules.driveControllerType == Controller.ControllerType.TalonFX && swerveConstants.modules.steerControllerType == Controller.ControllerType.TalonFX;
        if (swerveConstants.special.enableOdometryThread && isTalonFX) {
            positionQueue = OdometryThread.getInstance().registerSignal(((TalonFXController) driveMotor).getPositionSignal(swerveConstants.special.odometryThreadFrequency).clone());
            angleQueue = OdometryThread.getInstance().registerSignal(((TalonFXController) steerMotor).getPositionSignal(swerveConstants.special.odometryThreadFrequency).clone());
            timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        }
    }

    /** {@inheritDoc} */
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean preventJittering) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, Rotation2d.fromRadians(steerMotor.getPosition()));
        this.desiredState = desiredState;

        //Drive
        if (isOpenLoop)
            driveMotor.setPercent(desiredState.speedMetersPerSecond / swerveConstants.speeds.maxSpeed);
        else
            driveMotor.setVelocity(desiredState.speedMetersPerSecond);

        //Angle
        Rotation2d angle = desiredState.angle;
        if (preventJittering) {
            // Prevent rotating module if speed is less than 1%. Prevents jittering.
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (swerveConstants.speeds.maxSpeed * swerveConstants.modules.jitterPreventionPercent)) ? lastAngle : desiredState.angle;
        }
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - steerMotor.getPosition(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(steerMotor.getPosition() + error);
        //Rotate
        steerMotor.setPosition(angle.getRadians());
        lastAngle = angle;
    }

    /**
     * Re-seeds the steer motor's relative encoder from the CANCoder's absolute position, wrapped to
     * {@code [-pi, pi)}. Use this to recover a correct steer angle after a brownout or power cycle
     * without re-homing the module by hand.
     */
    public void resetToAbsolute() {
        double absolutePosition = ((getCANCoder().getRadians() + Math.PI * 3) % (Math.PI * 2)) - Math.PI;

        System.out.println("Encoder: " + steerMotor.getPosition() + " -> Absolute: " + absolutePosition);
        steerMotor.setEncoder(absolutePosition);
    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * {@inheritDoc}
     * <p>
     * The absolute CANCoder position is only re-read every {@value #ABSOLUTE_POSITION_UPDATE_PERIOD}
     * calls (cached otherwise), since CAN reads of it are comparatively expensive.
     */
    public SwerveModuleIOInputs update() {
        SwerveModuleIOInputs inputs = new  SwerveModuleIOInputs();

        inputs.moduleNumber = moduleNumber;
        inputs.state = new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromRadians(steerMotor.getPosition()));
        inputs.desiredState = desiredState;
        inputs.position = new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromRadians(steerMotor.getPosition()));
        if (absolutePositionUpdateCounter++ >= ABSOLUTE_POSITION_UPDATE_PERIOD) {
            absolutePositionUpdateCounter = 0;
            cachedAbsolutePosition = getCANCoder();
        }
        inputs.absolutePosition = cachedAbsolutePosition;

        if (swerveConstants.special.enableOdometryThread && isTalonFX) {
            inputs.positions = positionQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.angles = angleQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
            inputs.timestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

            positionQueue.clear();
            angleQueue.clear();
            timestampQueue.clear();
        }

        return inputs;
    }

    /** {@inheritDoc} Delegates to the drive and steer {@link Controller}s' own periodic updates. */
    @Override
    public void periodic() {
        driveMotor.periodic();
        steerMotor.periodic();
    }
}
