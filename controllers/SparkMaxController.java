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

        main = new SparkMax(constants.main.id, SparkMax.MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(constants.main.inverted);
		config.smartCurrentLimit((int)constants.currentLimit);

		config.softLimit.forwardSoftLimit(constants.maxSoftLimit != Double.MAX_VALUE ? constants.maxSoftLimit : 0)
			.reverseSoftLimit(constants.minSoftLimit != Double.MIN_VALUE ? constants.minSoftLimit : 0)
            .forwardSoftLimitEnabled(constants.maxSoftLimit != Double.MAX_VALUE)
            .reverseSoftLimitEnabled(constants.minSoftLimit != Double.MIN_VALUE);

		config.closedLoop.pid(constants.controlConstants.P, constants.controlConstants.I, constants.controlConstants.D);

        config.encoder.positionConversionFactor(constants.conversionFactor)
            .velocityConversionFactor(constants.conversionFactor / 60);

        main.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        followers = new SparkMax[constants.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new SparkMax(constants.followers[i].id, SparkMax.MotorType.kBrushless);

			SparkMaxConfig followerConfig = new SparkMaxConfig();
            followerConfig.follow(main, constants.followers[i].inverted);
            followers[i].configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
		}

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.controlConstants.cruiseVelocity, constants.controlConstants.acceleration));

        profiledPIDController = new ProfiledPIDController(
				constants.controlConstants.P,
				constants.controlConstants.I,
				constants.controlConstants.D,
				new TrapezoidProfile.Constraints(
                    constants.controlConstants.cruiseVelocity, constants.controlConstants.acceleration));
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

        main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        if (constants.controlConstants.type == SmartControlType.PID)
            main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kPosition);

        profiledPIDController.setGoal(position);
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        if (constants.controlConstants.type == SmartControlType.PID)
            main.getClosedLoopController().setReference(getGoal(), SparkBase.ControlType.kVelocity);

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
        switch (constants.controlConstants.type) {
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
