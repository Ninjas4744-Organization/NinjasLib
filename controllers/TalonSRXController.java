package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

public class TalonSRXController extends Controller {
    private final TalonSRX main;
    private final TalonSRX[] followers;
    private double lastVelocity;

    public TalonSRXController(RealControllerConstants constants) {
		super(constants);

        main = new TalonSRX(constants.main.id);
        main.configFactoryDefault();
        main.setInverted(constants.main.inverted);
        main.configPeakCurrentLimit((int) constants.currentLimit);

        main.config_kP(0, constants.controlConstants.P);
        main.config_kI(0, constants.controlConstants.I);
        main.config_kD(0, constants.controlConstants.D);
        main.configMotionCruiseVelocity(
            constants.controlConstants.cruiseVelocity * constants.conversionFactor / 10);
        main.configMotionAcceleration(constants.controlConstants.acceleration * constants.conversionFactor / 10);
        main.configForwardSoftLimitEnable(constants.maxSoftLimit != Double.MAX_VALUE);
        main.configReverseSoftLimitEnable(constants.minSoftLimit != Double.MIN_VALUE);
        main.configForwardSoftLimitThreshold(constants.maxSoftLimit);
        main.configReverseSoftLimitThreshold(constants.minSoftLimit);

        followers = new TalonSRX[constants.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonSRX(constants.followers[i].id);
            followers[i].configFactoryDefault();
            followers[i].follow(main);
            followers[i].setInverted(constants.followers[i].inverted ^ constants.main.inverted);
		}
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

        main.set(TalonSRXControlMode.PercentOutput, percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        switch (constants.controlConstants.type) {
			case PROFILE, PROFILED_PID:
                main.set(TalonSRXControlMode.MotionMagic, position / constants.conversionFactor);
				break;

			case PID:
                main.set(TalonSRXControlMode.Position, position / constants.conversionFactor);
				break;
		}
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        switch (constants.controlConstants.type) {
			case PROFILED_PID:
                main.set(TalonSRXControlMode.MotionMagic, velocity / constants.conversionFactor);
				break;

			case PID:
                main.set(TalonSRXControlMode.Velocity, velocity / constants.conversionFactor);
				break;

			case PROFILE:
				throw new UnsupportedOperationException("Velocity profile control not supported on TalonSRX");
		}
	}

	@Override
	public void stop() {
        main.set(TalonSRXControlMode.PercentOutput, 0);
	}

	@Override
	public double getPosition() {
        return main.getSelectedSensorPosition() * constants.conversionFactor;
	}

	@Override
	public double getVelocity() {
        return main.getSelectedSensorVelocity() * constants.conversionFactor;
    }

    @Override
    public double getAcceleration() {
        double acc = (getVelocity() - lastVelocity) / 0.02;
        lastVelocity = getVelocity();
        return acc;
	}

	@Override
	public double getOutput() {
        return main.getMotorOutputPercent();
	}

	@Override
	public double getCurrent() {
        return main.getBusVoltage();
	}

	@Override
	public void setEncoder(double position) {
        main.setSelectedSensorPosition(position / constants.conversionFactor);
	}
}
