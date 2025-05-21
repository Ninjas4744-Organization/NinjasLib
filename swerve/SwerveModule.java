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
import frc.lib.NinjasLib.controllers.SparkMaxController;
import frc.lib.NinjasLib.controllers.TalonFXController;
import frc.lib.NinjasLib.controllers.TalonSRXController;
import frc.lib.NinjasLib.dataclasses.SwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    public final int moduleNumber;

    private final Controller angleMotor;
    private final Controller driveMotor;

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

        if (constants.driveControllerType.equals(SparkMaxController.class))
            driveMotor = new SparkMaxController(constants.driveMotorConstants);
        else if (constants.driveControllerType.equals(TalonFXController.class))
            driveMotor = new TalonFXController(constants.driveMotorConstants);
        else if (constants.driveControllerType.equals(TalonSRXController.class))
            driveMotor = new TalonSRXController(constants.driveMotorConstants);
        /* else if (controllerClass.equals(NinjasSimulatedController.class))
            driveMotor = new NinjasSimulatedController(constants.driveMotorConstants);*/
        else
            throw new IllegalArgumentException("Invalid drive controller type: " + constants.driveControllerType.getSimpleName());

        if (constants.angleControllerType.equals(SparkMaxController.class))
            angleMotor = new SparkMaxController(constants.angleMotorConstants);
        else if (constants.angleControllerType.equals(TalonFXController.class))
            angleMotor = new TalonFXController(constants.angleMotorConstants);
        else if (constants.angleControllerType.equals(TalonSRXController.class))
            angleMotor = new TalonSRXController(constants.angleMotorConstants);
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
        desiredState = SwerveUtils.optimizeModuleState(desiredState, getState().angle);

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
