package frc.lib.NinjasLib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.util.DerivativeCalculator;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;

/**
 * A {@link Controller} implementation for simulation: instead of talking to real motor hardware,
 * it drives a WPILib {@link LinearSystemSim} (an {@link ElevatorSim}, {@link SingleJointedArmSim},
 * {@link DCMotorSim}, {@link FlywheelSim}, or a custom subclass, supplied via
 * {@link ControllerConstants#simSystem}) and runs its own {@link PIDController}/
 * {@link ProfiledPIDController}/{@link TrapezoidProfile} in software to emulate the closed-loop
 * behavior real motor controller firmware would otherwise provide. Created automatically by
 * {@link Controller#createController(ControllerType, ControllerConstants)} when not running on a
 * real robot.
 */
public class SimulatedController extends Controller {
    private enum SimType {
        ELEVATOR,
        ARM,
        DC_MOTOR,
        FLYWHEEL,
        UNKNOWN
    }

    /**
     * Can be any LinearSystemSim subclass - ElevatorSim, SingleJointedArmSim, DCMotorSim,
     * FlywheelSim, or a custom one. Position/velocity/current access is dispatched based on the
     * actual runtime type, since the output shape differs between mechanisms (e.g. FlywheelSim has
     * no position state). The type is checked once here and cached, rather than re-checked with
     * instanceof (and re-logged if unknown) on every call.
     */
    private final LinearSystemSim<?, ?, ?> sim;
    private final SimType simType;

    private DerivativeCalculator accelerationCalculator;

    private final TrapezoidProfile profile;
    private final ProfiledPIDController profiledPIDController;
    private final PIDController PIDController;
    private boolean isCurrentlyProfiling = false;

    /**
     * Builds the simulated mechanism from {@link ControllerConstants#simSystem} (detecting which
     * {@link LinearSystemSim} subclass it is so position/velocity/current reads can be dispatched
     * correctly), and sets up the software trapezoid profile and PID controllers used to emulate
     * closed-loop control per {@link ControllerConstants#real}'s {@code controlConstants}.
     *
     * @param constants the controller configuration, including the simulated mechanism supplier
     */
    public SimulatedController(ControllerConstants constants) {
        super(constants.real);

        sim = constants.simSystem.get();

        if (sim instanceof ElevatorSim)
            simType = SimType.ELEVATOR;
        else if (sim instanceof SingleJointedArmSim)
            simType = SimType.ARM;
        else if (sim instanceof DCMotorSim)
            simType = SimType.DC_MOTOR;
        else if (sim instanceof FlywheelSim)
            simType = SimType.FLYWHEEL;
        else {
            simType = SimType.UNKNOWN;
            NinjasLogger.logEventImportant("[SimulatedController] Unknown LinearSystemSim subclass, position/velocity/current/encoder will read as 0: " + sim.getClass().getSimpleName());
        }

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            constants.real.control.controlConstants.cruiseVelocity,
            constants.real.control.controlConstants.acceleration));

        profiledPIDController = new ProfiledPIDController(
            constants.real.control.controlConstants.P,
            constants.real.control.controlConstants.I,
            constants.real.control.controlConstants.D,
            new TrapezoidProfile.Constraints(
                constants.real.control.controlConstants.cruiseVelocity, constants.real.control.controlConstants.acceleration));
        profiledPIDController.setIZone(constants.real.control.controlConstants.IZone);

        PIDController = new PIDController(
            constants.real.control.controlConstants.P,
            constants.real.control.controlConstants.I,
            constants.real.control.controlConstants.D
        );
        PIDController.setIZone(constants.real.control.controlConstants.IZone);

        accelerationCalculator = new DerivativeCalculator(3);
    }

    private void setInputVoltage(double volts) {
        sim.setInput(volts);
    }

    /**
     * Treats {@code percent} as a fraction of the nominal 12V bus and applies that voltage
     * directly as the sim's input.
     *
     * @param percent how much to power the motor, between -1 and 1
     */
    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        setInputVoltage(percent * 12);
    }

    /**
     * Sets the goal for both the software profiled and unprofiled PID controllers; which one
     * actually drives the sim is chosen in {@link #periodic()} based on the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}.
     *
     * @param position the wanted position
     */
    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        profiledPIDController.setGoal(position);
        PIDController.setSetpoint(position);
    }

    /**
     * Sets the goal for both the software profiled and unprofiled PID controllers; which one
     * actually drives the sim is chosen in {@link #periodic()} based on the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}.
     *
     * @param velocity the wanted velocity
     */
    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        profiledPIDController.setGoal(velocity);
        PIDController.setSetpoint(velocity);
    }

    /** Stops the mechanism by driving the simulated input voltage to zero. */
    @Override
    public void stop() {
        super.stop();
        setInputVoltage(0);
    }

    /**
     * @return the sim's position output, or {@code 0} for a {@link FlywheelSim} (which has no
     * position state) or an unrecognized {@link LinearSystemSim} subclass
     */
    @Override
    public double getPosition() {
        // FlywheelSim only tracks velocity - no position state exists
        if (simType == SimType.FLYWHEEL || simType == SimType.UNKNOWN)
            return 0;

        return sim.getOutput(0);
    }

    /**
     * @return the sim's velocity, read via {@link FlywheelSim#getAngularVelocityRadPerSec()} for
     * a flywheel (whose output vector has no dedicated velocity index) or {@code 0} for an
     * unrecognized {@link LinearSystemSim} subclass
     */
    @Override
    public double getVelocity() {
        if (simType == SimType.UNKNOWN)
            return 0;

        if (simType == SimType.FLYWHEEL)
            return ((FlywheelSim) sim).getAngularVelocityRadPerSec();

        return sim.getOutput(1);
    }

    /** @return the acceleration estimated from recent velocity samples by {@link #accelerationCalculator} */
    @Override
    public double getAcceleration() {
        return accelerationCalculator.get();
    }

    /** @return the sim's input voltage expressed as a fraction of the nominal 12V bus */
    @Override
    public double getOutput() {
        return sim.getInput(0) / 12;
    }

    /** @return the sim's current draw, in amps; simulation makes no supply/stator distinction */
    @Override
    public double getSupplyCurrent() {
        return getCurrentDrawAmps();
    }

    /** @return the sim's current draw, in amps; simulation makes no supply/stator distinction */
    @Override
    public double getStatorCurrent() {
        return getCurrentDrawAmps();
    }

    private double getCurrentDrawAmps() {
        return switch (simType) {
            case ELEVATOR -> ((ElevatorSim) sim).getCurrentDrawAmps();
            case ARM -> ((SingleJointedArmSim) sim).getCurrentDrawAmps();
            case DC_MOTOR -> ((DCMotorSim) sim).getCurrentDrawAmps();
            case FLYWHEEL -> ((FlywheelSim) sim).getCurrentDrawAmps();
            case UNKNOWN -> 0;
        };
    }

    /**
     * Forces the sim's internal state to the given position (keeping its current velocity). Has
     * no effect on a {@link FlywheelSim} or an unrecognized {@link LinearSystemSim} subclass,
     * since neither exposes a position state to set.
     *
     * @param position the position to set the sim's encoder to
     */
    @Override
    public void setEncoder(double position) {
        switch (simType) {
            case ELEVATOR -> ((ElevatorSim) sim).setState(position, getVelocity());
            case ARM -> ((SingleJointedArmSim) sim).setState(position, getVelocity());
            case DC_MOTOR -> ((DCMotorSim) sim).setState(position, getVelocity());
            case FLYWHEEL, UNKNOWN -> {} // no position state to set, or unsupported sim type
        }
    }

    /**
     * Advances the simulation by one 20ms loop: since a simulated mechanism has no real motor
     * controller firmware running its closed loop, this method performs that role in software -
     * computing an input voltage from the profiled PID controller, plain PID controller, or
     * trapezoid profile depending on the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType},
     * applying it, clamping the mechanism to its configured soft limits, updating the
     * acceleration estimate, and stepping the underlying {@link LinearSystemSim}. Call this from
     * the owning subsystem's {@code simulationPeriodic()}.
     */
    @Override
    public void periodic() {
        switch (constants.control.controlConstants.type) {
            case PROFILED_PIDF:
                isCurrentlyProfiling = true;

                if (controlState == ControlState.POSITION)
                    setInputVoltage(profiledPIDController.calculate(getPosition()));
                else if (controlState == ControlState.VELOCITY)
                    setInputVoltage(constants.control.controlConstants.V * getGoal() + profiledPIDController.calculate(getVelocity()));
                break;

            case PIDF, TORQUE_CURRENT:
                if (controlState == ControlState.POSITION)
                    setInputVoltage(PIDController.calculate(getPosition()));
                else if (controlState == ControlState.VELOCITY)
                    setInputVoltage(constants.control.controlConstants.V * getGoal() + PIDController.calculate(getVelocity()));
                break;

            case PROFILE:
                if (controlState == ControlState.POSITION)
                    setInputVoltage(profile.calculate(
                        0.02,
                        new TrapezoidProfile.State(getPosition(), getVelocity()),
                        new TrapezoidProfile.State(getGoal(), 0))
                        .velocity * constants.control.controlConstants.V);
                else if (controlState == ControlState.VELOCITY)
                    setInputVoltage(profile.calculate(
                        0.02,
                        new TrapezoidProfile.State(getPosition(), getVelocity()),
                        new TrapezoidProfile.State(getPosition(), getGoal()))
                        .velocity * constants.control.controlConstants.V);
                break;
        }

        if (!isCurrentlyProfiling && controlState != ControlState.PERCENT_OUTPUT)
            profiledPIDController.reset(new TrapezoidProfile.State(getPosition(), getVelocity()));
        isCurrentlyProfiling = false;

        if (getPosition() >= constants.softLimits.max) {
//            stop();
            setEncoder(constants.softLimits.max);
        }

        if (getPosition() <= constants.softLimits.min) {
//            stop();
            setEncoder(constants.softLimits.min);
        }

        accelerationCalculator.calculate(getVelocity());
        sim.update(0.02);
        super.periodic();
    }
}