package frc.lib.NinjasLib.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.TalonFXController;
import frc.lib.NinjasLib.localization.OdometryThread;
import frc.lib.NinjasLib.swerve.SwerveUtils;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;

import java.util.Queue;

public class SwerveModuleIOReal implements SwerveModuleIO {
    public final int moduleNumber;
    private SwerveModuleConstants constants;
    private SwerveConstants swerveConstants;

    private final Controller steerMotor;
    private final Controller driveMotor;

    private Rotation2d lastAngle;
    private final CANcoder canCoder;

    private final boolean isTalonFX;
    private Queue<Double> positionQueue;
    private Queue<Double> angleQueue;
    private Queue<Double> timestampQueue;

    public SwerveModuleIOReal(SwerveModuleConstants constants, SwerveConstants swerveConstants) {
        moduleNumber = constants.moduleNumber;
        this.constants = constants;
        this.swerveConstants = swerveConstants;

        canCoder = new CANcoder(constants.canCoderID);
        canCoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withSensorDirection(constants.invertCANCoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(constants.CANCoderOffset)
        );

        driveMotor = Controller.createController(constants.driveControllerType, constants.driveMotorConstants);
        steerMotor = Controller.createController(constants.steerControllerType, constants.angleMotorConstants);

        lastAngle = Rotation2d.fromRadians(steerMotor.getPosition());

        isTalonFX = constants.driveControllerType == Controller.ControllerType.TalonFX && constants.steerControllerType == Controller.ControllerType.TalonFX;
        if (swerveConstants.enableOdometryThread && isTalonFX) {
            positionQueue = OdometryThread.getInstance().registerSignal(((TalonFXController) driveMotor).getPositionSignal().clone());
            angleQueue = OdometryThread.getInstance().registerSignal(((TalonFXController) steerMotor).getPositionSignal().clone());
            timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        }
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, Rotation2d.fromRadians(steerMotor.getPosition()));

        //Drive
        if (isOpenLoop) driveMotor.setPercent(desiredState.speedMetersPerSecond / constants.maxModuleSpeed);
        else driveMotor.setVelocity(desiredState.speedMetersPerSecond);

        //Angle
        // Prevent rotating module if speed is less than 3%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (constants.maxModuleSpeed * 0.03)) ? lastAngle : desiredState.angle;
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

    public Rotation2d getCANCoder() {
        canCoder.getAbsolutePosition().refresh();
        return Rotation2d.fromRadians(canCoder.getAbsolutePosition().getValue().in(Units.Radians));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.ModuleNumber = moduleNumber;
        inputs.State = new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromRadians(steerMotor.getPosition()));
        inputs.Position = new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromRadians(steerMotor.getPosition()));
        inputs.AbsoluteAngle = getCANCoder();

        if (swerveConstants.enableOdometryThread && isTalonFX) {
            inputs.Positions = positionQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.Angles = angleQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
            inputs.Timestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

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
