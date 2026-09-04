package frc.lib.NinjasLib.controllers;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

/**
 * {@link Controller} implementation that wraps a REV SparkMax brushless motor controller (via the
 * REVLib {@link SparkMax} API). Configures current limiting, soft limits, closed-loop gains, and
 * encoder conversion factors from {@link RealControllerConstants} on construction, and follower
 * SparkMaxes to mirror the main one.
 */
public class SparkMaxController extends Controller {
    private final SparkMax main;
    private final SparkMax[] followers;

    private final TrapezoidProfile profile;
    private final ProfiledPIDController profiledPIDController;
	private boolean isCurrentlyPiding = false;
    private double lastVelocity;

    /**
     * Constructs and configures the main SparkMax (inversion, smart current limit, soft limits,
     * closed-loop PID gains, and encoder position/velocity conversion factors derived from the
     * configured gear ratio), then constructs each follower SparkMax to follow it.
     *
     * @param constants the controller configuration
     */
    public SparkMaxController(RealControllerConstants constants) {
		super(constants);

        main = new SparkMax(constants.base.main.id, SparkMax.MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(constants.base.main.inverted);
		config.smartCurrentLimit((int)constants.base.statorCurrentLimit);

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

	/**
	 * Drives the main SparkMax directly in open-loop percent output.
	 *
	 * @param percent how much to power the motor, between -1 and 1
	 */
	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

        main.set(percent);
	}

	/**
	 * If the control type is plain {@code PIDF}, commands the SparkMax's onboard closed-loop
	 * controller directly to the position setpoint; for profiled/other control types the software
	 * profile in {@link #periodic()} drives the motor instead, so this just updates its goal.
	 *
	 * @param position the wanted position
	 */
	@Override
	public void setPosition(double position) {
		super.setPosition(position);

        if (constants.control.controlConstants.type == ControlType.PIDF)
            main.getClosedLoopController().setSetpoint(getGoal(), SparkBase.ControlType.kPosition);

        profiledPIDController.setGoal(position);
	}

	/**
	 * If the control type is plain {@code PIDF}, commands the SparkMax's onboard closed-loop
	 * controller directly to the velocity setpoint; for profiled/other control types the software
	 * profile in {@link #periodic()} drives the motor instead, so this just updates its goal.
	 *
	 * @param velocity the wanted velocity
	 */
	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

        if (constants.control.controlConstants.type == ControlType.PIDF)
            main.getClosedLoopController().setSetpoint(getGoal(), SparkBase.ControlType.kVelocity);

        profiledPIDController.setGoal(velocity);
	}

	/** Stops the main SparkMax (and its followers) via {@link SparkMax#stopMotor()}. */
	@Override
	public void stop() {
		super.stop();
        main.stopMotor();
	}

	/** @return the main SparkMax's built-in encoder position, in the units set by the configured position conversion factor */
	@Override
	public double getPosition() {
        return main.getEncoder().getPosition();
	}

	/** @return the main SparkMax's built-in encoder velocity, in the units set by the configured velocity conversion factor */
	@Override
	public double getVelocity() {
        return main.getEncoder().getVelocity();
    }

    /** @return the acceleration computed as the finite-difference change in {@link #getVelocity()} over one 20ms loop */
    @Override
    public double getAcceleration() {
        double acc = (getVelocity() - lastVelocity) / 0.02;
        lastVelocity = getVelocity();
        return acc;
	}

	/** @return the effective applied motor output as a fraction of the nominal 12V bus (bus voltage times duty cycle, divided by 12) */
	@Override
	public double getOutput() {
        return main.getBusVoltage() * main.getAppliedOutput() / 12;
	}

	/**
	 * REVLib doesn't expose battery/supply current directly, so this estimates it as the stator
	 * (output) current scaled by the duty cycle.
	 *
	 * @return the estimated current drawn from the battery, in amps
	 */
	@Override
	public double getSupplyCurrent() {
        return main.getOutputCurrent() * Math.abs(main.getAppliedOutput()); // estimated: stator * duty cycle
	}

	/** @return the main SparkMax's output (stator) current, in amps */
	@Override
	public double getStatorCurrent() {
        return main.getOutputCurrent();
	}

	/**
	 * Overwrites the main SparkMax's built-in encoder position.
	 *
	 * @param position the position to set the encoder to
	 */
	@Override
	public void setEncoder(double position) {
        main.getEncoder().setPosition(position);
	}

	/**
	 * For {@code PROFILED_PIDF} and {@code PROFILE} control types, computes one step of the
	 * software trapezoid profile / profiled PID controller each loop and applies the result to
	 * the motor as a voltage-derived percent output (since the SparkMax has no built-in Motion
	 * Magic-style profiling); {@code PIDF} control is instead driven directly by the SparkMax's
	 * onboard closed loop in {@link #setPosition(double)}/{@link #setVelocity(double)}. Call this
	 * from the owning subsystem's {@code periodic()}.
	 */
	@Override
	public void periodic() {
        switch (constants.control.controlConstants.type) {
			case PROFILED_PIDF:
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
