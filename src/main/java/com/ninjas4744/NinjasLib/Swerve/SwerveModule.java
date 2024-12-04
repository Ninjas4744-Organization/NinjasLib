package com.ninjas4744.NinjasLib.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ninjas4744.NinjasLib.Controllers.NinjasController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.DataClasses.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SwerveModule {
	public final int moduleNumber;

	private final NinjasController angleMotor;
	private final NinjasController driveMotor;

	private Rotation2d lastAngle;
	private final CANcoder canCoder;
	private final double maxModuleSpeed;

	public SwerveModule(SwerveModuleConstants constants) {
		moduleNumber = constants.moduleNumber;
		maxModuleSpeed = constants.maxModuleSpeed;
		
		canCoder = new CANcoder(constants.canCoderID);
		angleMotor = new NinjasSparkMaxController(constants.angleMotorConstants);
		resetToAbsolute();
		lastAngle = Rotation2d.fromDegrees(angleMotor.getPosition());
		
		driveMotor = new NinjasSparkMaxController(constants.driveMotorConstants);

		Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Speed", () -> getState().speedMetersPerSecond);
		Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Angle", () -> getState().angle.getDegrees());
		Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Position", () -> getPosition().distanceMeters);
		Shuffleboard.getTab("Swerve Mod " + moduleNumber).addNumber("Absolute Angle", () -> getCanCoder().getDegrees());
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
		// Prevent rotating module if speed is less then 1%. Prevents Jittering.
		Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxModuleSpeed * 0.01)) ? lastAngle : desiredState.angle;
		angleMotor.setPosition(angle.getDegrees());
		lastAngle = angle;
	}

	public void resetToAbsolute() {
		double absolutePosition = getCanCoder().getDegrees() % 360;
		double currentAngle = angleMotor.getPosition();

		double angleDiff = ((absolutePosition - currentAngle + 540) % 360) - 180;  // Normalize to [-180, 180]
		double targetAngle = currentAngle + angleDiff;

		System.out.println("Encoder: " + angleMotor.getPosition() + " -> Absolute: " + targetAngle);
		if (Math.abs(angleDiff) > 2) angleMotor.setEncoder(targetAngle);
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
}
