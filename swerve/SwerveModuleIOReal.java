package frc.lib.NinjasLib.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.dataclasses.SwerveModuleConstants;

public class SwerveModuleIOReal implements SwerveModuleIO {
    public final int moduleNumber;

    private final Controller steerMotor;
    private final Controller driveMotor;

    private Rotation2d lastAngle;
    private final CANcoder canCoder;
    private final double maxModuleSpeed;

    public SwerveModuleIOReal(SwerveModuleConstants constants) {
        moduleNumber = constants.moduleNumber;
        maxModuleSpeed = constants.maxModuleSpeed;

        canCoder = new CANcoder(constants.canCoderID);
        canCoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor
                .withSensorDirection(constants.invertCANCoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(constants.CANCoderOffset)
        );

//        switch (constants.driveControllerType) {
//            case SparkMax:
//                driveMotor = new SparkMaxController(constants.driveMotorConstants);
//                break;
//
//            case TalonSRX:
//                driveMotor = new TalonSRXController(constants.driveMotorConstants);
//                break;
//
//            default:
//                driveMotor = new TalonFXController(constants.driveMotorConstants);
//                break;
//        }

        driveMotor = Controller.createController(constants.driveControllerType, constants.driveMotorConstants);
        steerMotor = Controller.createController(constants.angleControllerType, constants.angleMotorConstants);

//        switch (constants.angleControllerType) {
//            case SparkMax:
//                steerMotor = new SparkMaxController(constants.angleMotorConstants);
//                break;
//
//            case TalonSRX:
//                steerMotor = new TalonSRXController(constants.angleMotorConstants);
//                break;
//
//            default:
//                steerMotor = new TalonFXController(constants.angleMotorConstants);
//                break;
//        }

        lastAngle = Rotation2d.fromRadians(steerMotor.getPosition());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromRadians(steerMotor.getPosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromRadians(steerMotor.getPosition()));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveUtils.optimizeModuleState(desiredState, getState().angle);

        //Drive
        if (isOpenLoop) driveMotor.setPercent(desiredState.speedMetersPerSecond / maxModuleSpeed);
        else driveMotor.setVelocity(desiredState.speedMetersPerSecond);

        //Angle
        // Prevent rotating module if speed is less than 3%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.03)) ? lastAngle : desiredState.angle;
        //Prevent jumping from -180 to 180
        double errorBound = (Math.PI - -Math.PI) / 2.0;
        double error = MathUtil.inputModulus(angle.getRadians() - steerMotor.getPosition(), -errorBound, errorBound);
        angle = Rotation2d.fromRadians(steerMotor.getPosition() + error);
        //Rotate
        steerMotor.setPosition(angle.getRadians());
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = ((getCanCoder().getRadians() + Math.PI * 3) % (Math.PI * 2)) - Math.PI;
//        double currentAngle = angleMotor.getPosition();

//        double angleDiff = ((absolutePosition - currentAngle + 540) % 360) - 180;  // Normalize to [-180, 180]
//        double targetAngle = currentAngle + angleDiff;

        System.out.println("Encoder: " + steerMotor.getPosition() + " -> Absolute: " + absolutePosition);
        steerMotor.setEncoder(absolutePosition);
    }

    public Rotation2d getCanCoder() {
        canCoder.getAbsolutePosition().refresh();
        return Rotation2d.fromRadians(canCoder.getAbsolutePosition().getValue().in(Units.Radians));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.Speed = getState().speedMetersPerSecond;
        inputs.Angle = getState().angle;
        inputs.AbsoluteAngle = getCanCoder();
    }

    @Override
    public int getModuleNumber() {
        return moduleNumber;
    }

    @Override
    public void periodic() {
        driveMotor.periodic();
        steerMotor.periodic();
    }
}
