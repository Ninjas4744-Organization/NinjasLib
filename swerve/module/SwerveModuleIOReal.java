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

public class SwerveModuleIOReal implements SwerveModuleIO {
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

    // Cached arrays to avoid GC pressure from stream operations — reused when size matches
    private double[] positionArray = new double[0];
    private Rotation2d[] angleArray = new Rotation2d[0];
    private double[] timestampArray = new double[0];

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

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean preventJittering) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, Rotation2d.fromRadians(steerMotor.getPosition()));
        this.desiredState = desiredState;

        //Drive
        if (isOpenLoop)
            driveMotor.setPercent(desiredState.speedMetersPerSecond / swerveConstants.limits.maxSpeed);
        else
            driveMotor.setVelocity(desiredState.speedMetersPerSecond);

        //Angle
        Rotation2d angle = desiredState.angle;
        if (preventJittering) {
            // Prevent rotating module if speed is less than 1%. Prevents jittering.
            angle = (Math.abs(desiredState.speedMetersPerSecond) <= (swerveConstants.limits.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;
        }
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - steerMotor.getPosition(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(steerMotor.getPosition() + error);
        //Rotate
        steerMotor.setPosition(angle.getRadians());
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = ((getCANCoder().getRadians() + Math.PI * 3) % (Math.PI * 2)) - Math.PI;

        System.out.println("Encoder: " + steerMotor.getPosition() + " -> Absolute: " + absolutePosition);
        steerMotor.setEncoder(absolutePosition);
    }

    private Rotation2d getCANCoder() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.ModuleNumber = moduleNumber;
        inputs.State = new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromRadians(steerMotor.getPosition()));
        inputs.DesiredState = desiredState;
        inputs.Position = new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromRadians(steerMotor.getPosition()));
        if (absolutePositionUpdateCounter++ >= ABSOLUTE_POSITION_UPDATE_PERIOD) {
            absolutePositionUpdateCounter = 0;
            cachedAbsolutePosition = getCANCoder();
        }
        inputs.AbsolutePosition = cachedAbsolutePosition;

        if (swerveConstants.special.enableOdometryThread && isTalonFX) {
            int size = positionQueue.size();

            if (positionArray.length != size) positionArray = new double[size];
            if (angleArray.length != size) angleArray = new Rotation2d[size];
            if (timestampArray.length != size) timestampArray = new double[size];

            int idx = 0;
            for (Double val : positionQueue) positionArray[idx++] = val;

            idx = 0;
            for (Double val : angleQueue) angleArray[idx++] = Rotation2d.fromRadians(val);

            idx = 0;
            for (Double val : timestampQueue) timestampArray[idx++] = val;

            inputs.Positions = positionArray;
            inputs.Angles = angleArray;
            inputs.Timestamps = timestampArray;

            positionQueue.clear();
            angleQueue.clear();
            timestampQueue.clear();
        }
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
        steerMotor.periodic();
    }
}
