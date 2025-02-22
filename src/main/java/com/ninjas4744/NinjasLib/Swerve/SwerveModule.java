package com.ninjas4744.NinjasLib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ninjas4744.NinjasLib.Controllers.*;
import com.ninjas4744.NinjasLib.DataClasses.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    public final int moduleNumber;

    private final NinjasController angleMotor;
    private final NinjasController driveMotor;

    private Rotation2d lastAngle;
    private final CANcoder canCoder;
    private final double maxModuleSpeed;

    private SwerveModuleConstants _constants;

    public SwerveModule(SwerveModuleConstants constants) {
        _constants = constants;

        moduleNumber = constants.moduleNumber;
        maxModuleSpeed = constants.maxModuleSpeed;

        canCoder = new CANcoder(constants.canCoderID);
		canCoder.getConfigurator().apply(
                new CANcoderConfiguration().MagnetSensor
                .withSensorDirection(constants.invertCANCoder ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(constants.CANCoderOffset)
        );

        if (constants.driveControllerType.equals(NinjasSparkMaxController.class))
            driveMotor = new NinjasSparkMaxController(constants.driveMotorConstants);
        else if (constants.driveControllerType.equals(NinjasTalonFXController.class))
            driveMotor = new NinjasTalonFXController(constants.driveMotorConstants);
        else if (constants.driveControllerType.equals(NinjasTalonSRXController.class))
            driveMotor = new NinjasTalonSRXController(constants.driveMotorConstants);
        /* else if (controllerClass.equals(NinjasSimulatedController.class))
            driveMotor = new NinjasSimulatedController(constants.driveMotorConstants);*/
        else
            throw new IllegalArgumentException("Invalid drive controller type: " + constants.driveControllerType.getSimpleName());

        if (constants.angleControllerType.equals(NinjasSparkMaxController.class))
            angleMotor = new NinjasSparkMaxController(constants.angleMotorConstants);
        else if (constants.angleControllerType.equals(NinjasTalonFXController.class))
            angleMotor = new NinjasTalonFXController(constants.angleMotorConstants);
        else if (constants.angleControllerType.equals(NinjasTalonSRXController.class))
            angleMotor = new NinjasTalonSRXController(constants.angleMotorConstants);
        /* else if (controllerClass.equals(NinjasSimulatedController.class))
            angleMotor = new NinjasSimulatedController(constants.angleMotorConstants);*/
        else
            throw new IllegalArgumentException("Invalid angle controller type: " + constants.driveControllerType.getSimpleName());

        lastAngle = Rotation2d.fromDegrees(angleMotor.getPosition());

//        if(!constants.createShuffleboard)
//            return;
//
//        Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Speed", () -> getState().speedMetersPerSecond);
//        Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Angle", () -> getState().angle.getDegrees());
//        Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Absolute Angle", () -> getCanCoder().getDegrees());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), Rotation2d.fromDegrees(angleMotor.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = optimize(desiredState, getState().angle);

        //Drive
        if (isOpenLoop) driveMotor.setPercent(desiredState.speedMetersPerSecond / maxModuleSpeed);
        else driveMotor.setVelocity(desiredState.speedMetersPerSecond);

        //Angle
        // Prevent rotating module if speed is less than 3%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.03)) ? lastAngle : desiredState.angle;
        //Prevent jumping from -180 to 180
        double errorBound = (180 - -180) / 2.0;
        double error = MathUtil.inputModulus(angle.getDegrees() - angleMotor.getPosition(), -errorBound, errorBound);
        angle = Rotation2d.fromDegrees(angleMotor.getPosition() + error);
        //Rotate
        angleMotor.setPosition(angle.getDegrees());
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = ((getCanCoder().getDegrees() + 540) % 360) - 180;
//        double currentAngle = angleMotor.getPosition();

//        double angleDiff = ((absolutePosition - currentAngle + 540) % 360) - 180;  // Normalize to [-180, 180]
//        double targetAngle = currentAngle + angleDiff;

        System.out.println("Encoder: " + angleMotor.getPosition() + " -> Absolute: " + absolutePosition);
        angleMotor.setEncoder(absolutePosition);
    }

    public Rotation2d getCanCoder() {
        canCoder.getAbsolutePosition().refresh();
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValue().in(Units.Degrees));
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double currentDegrees = currentAngle.getDegrees();
        double targetDegrees = desiredState.angle.getDegrees();

        double delta = targetDegrees - currentDegrees;
        delta = (delta + 360) % 360;  // Normalize delta to [0, 360)

        if (delta > 180) delta -= 360;  // Adjust to [-180, 180)

        if (Math.abs(delta) > 90) {
            targetDegrees += delta > 0 ? -180 : 180;
            desiredState = new SwerveModuleState(-desiredState.speedMetersPerSecond, Rotation2d.fromDegrees(targetDegrees));
        }

        return new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromDegrees((targetDegrees + 360) % 360));
    }

    public void periodic() {
        driveMotor.periodic();
        angleMotor.periodic();

        if(!_constants.enableLogging)
            return;

        Logger.recordOutput("Swerve Module " + moduleNumber + "/Speed", getState().speedMetersPerSecond);
        Logger.recordOutput("Swerve Module " + moduleNumber + "/Angle", getState().angle.getDegrees());
        Logger.recordOutput("Swerve Module " + moduleNumber + "/Absolute Angle", getCanCoder().getDegrees());
    }
}
