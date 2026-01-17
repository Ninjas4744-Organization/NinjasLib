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

        main = new TalonSRX(constants.base.main.id);
        main.configFactoryDefault();
        main.setInverted(constants.base.main.inverted);
        main.configPeakCurrentLimit((int) constants.base.currentLimit);

        main.config_kP(0, constants.control.controlConstants.P);
        main.config_kI(0, constants.control.controlConstants.I);
        main.config_kD(0, constants.control.controlConstants.D);
        main.configMotionCruiseVelocity(
            constants.control.controlConstants.cruiseVelocity * constants.control.conversionFactor / 10);
        main.configMotionAcceleration(constants.control.controlConstants.acceleration * constants.control.conversionFactor / 10);
        main.configForwardSoftLimitEnable(constants.softLimits.max != Double.POSITIVE_INFINITY);
        main.configReverseSoftLimitEnable(constants.softLimits.min != Double.NEGATIVE_INFINITY);
        main.configForwardSoftLimitThreshold(constants.softLimits.max);
        main.configReverseSoftLimitThreshold(constants.softLimits.min);

        followers = new TalonSRX[constants.base.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonSRX(constants.base.followers[i].id);
            followers[i].configFactoryDefault();
            followers[i].follow(main);
            followers[i].setInverted(constants.base.followers[i].inverted ^ constants.base.main.inverted);
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

        switch (constants.control.controlConstants.type) {
			case PROFILE, PROFILED_PID:
                main.set(TalonSRXControlMode.MotionMagic, position / constants.control.conversionFactor);
				break;

			case PID:
                main.set(TalonSRXControlMode.Position, position / constants.control.conversionFactor);
				break;
		}
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        switch (constants.control.controlConstants.type) {
			case PROFILED_PID:
                main.set(TalonSRXControlMode.MotionMagic, velocity / constants.control.conversionFactor);
				break;

			case PID:
                main.set(TalonSRXControlMode.Velocity, velocity / constants.control.conversionFactor);
				break;

			case PROFILE:
				throw new UnsupportedOperationException("Velocity profile control not supported on TalonSRX");
		}
	}

	@Override
	public void stop() {
		super.stop();
        main.set(TalonSRXControlMode.PercentOutput, 0);
	}

	@Override
	public double getPosition() {
        return main.getSelectedSensorPosition() * constants.control.conversionFactor;
	}

	@Override
	public double getVelocity() {
        return main.getSelectedSensorVelocity() * constants.control.conversionFactor;
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
        main.setSelectedSensorPosition(position / constants.control.conversionFactor);
	}
}
