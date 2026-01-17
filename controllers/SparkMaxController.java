package frc.lib.NinjasLib.controllers;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.NinjasLib.controllers.constants.ControlConstants.SmartControlType;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

public class SparkMaxController extends Controller {
    private final SparkMax main;
    private final SparkMax[] followers;

    private final TrapezoidProfile profile;
    private final ProfiledPIDController profiledPIDController;
	private boolean isCurrentlyPiding = false;
    private double lastVelocity;

    public SparkMaxController(RealControllerConstants constants) {
		super(constants);

        main = new SparkMax(constants.base.main.id, SparkMax.MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(constants.base.main.inverted);
		config.smartCurrentLimit((int)constants.base.currentLimit);

		config.softLimit.forwardSoftLimit(constants.softLimits.max != Double.POSITIVE_INFINITY ? constants.softLimits.max : 0)
			.reverseSoftLimit(constants.softLimits.min != Double.NEGATIVE_INFINITY ? constants.softLimits.min : 0)
            .forwardSoftLimitEnabled(constants.softLimits.max != Double.POSITIVE_INFINITY)
            .reverseSoftLimitEnabled(constants.softLimits.min != Double.NEGATIVE_INFINITY);

		config.closedLoop.pid(constants.control.controlConstants.P, constants.control.controlConstants.I, constants.control.controlConstants.D);

		config.encoder.positionConversionFactor(constants.control.conversionFactor / constants.control.gearRatio)
				.velocityConversionFactor(constants.control.conversionFactor / constants.control.gearRatio / 60);

        main.configure(config, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        followers = new SparkMax[constants.base.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new SparkMax(constants.base.followers[i].id, SparkMax.MotorType.kBrushless);

			SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.follow(main, constants.base.followers[i].inverted);
            followers[i].configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
		}

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.control.controlConstants.cruiseVelocity, constants.control.controlConstants.acceleration));

        profiledPIDController = new ProfiledPIDController(
				constants.control.controlConstants.P,
				constants.control.controlConstants.I,
				constants.control.controlConstants.D,
				new TrapezoidProfile.Constraints(constants.control.controlConstants.cruiseVelocity, constants.control.controlConstants.acceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

        main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        if (constants.control.controlConstants.type == SmartControlType.PID)
            main.getClosedLoopController().setSetpoint(getGoal(), SparkBase.ControlType.kPosition);

        profiledPIDController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        if (constants.control.controlConstants.type == SmartControlType.PID)
            main.getClosedLoopController().setSetpoint(getGoal(), SparkBase.ControlType.kVelocity);

        profiledPIDController.setGoal(velocity);
	}

	@Override
	public void stop() {
        main.stopMotor();
	}

	@Override
	public double getPosition() {
        return main.getEncoder().getPosition();
	}

	@Override
	public double getVelocity() {
        return main.getEncoder().getVelocity();
    }

    @Override
    public double getAcceleration() {
        double acc = (getVelocity() - lastVelocity) / 0.02;
        lastVelocity = getVelocity();
        return acc;
	}

	@Override
	public double getOutput() {
        return main.getBusVoltage() * main.getAppliedOutput() / 12;
	}

	@Override
	public double getCurrent() {
        return main.getOutputCurrent();
	}

	@Override
	public void setEncoder(double position) {
        main.getEncoder().setPosition(position);
	}

	@Override
	public void periodic() {
        switch (constants.control.controlConstants.type) {
			case PROFILED_PID:
				isCurrentlyPiding = true;

                if (controlState == ControlState.POSITION)
                    main.set(profiledPIDController.calculate(getPosition()) / 12);
                else if (controlState == ControlState.VELOCITY)
                    main.set(profiledPIDController.calculate(getVelocity()) / 12);
				break;

			case PROFILE:
                if (controlState == ControlState.POSITION)
                    main.set(profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getGoal(), 0))
						.velocity / 12);
                else if (controlState == ControlState.VELOCITY)
                    main.set(profile.calculate(
					0.02,
					new TrapezoidProfile.State(getPosition(), getVelocity()),
					new TrapezoidProfile.State(getPosition(), getGoal()))
						.velocity / 12);
				break;
		}

        if (!isCurrentlyPiding && controlState != ControlState.PERCENT_OUTPUT)
            profiledPIDController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
		isCurrentlyPiding = false;

		super.periodic();
	}
}
