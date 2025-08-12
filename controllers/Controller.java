package frc.lib.NinjasLib.controllers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NinjasLib.controllers.constants.ControlConstants.SmartControlType;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.robot.Robot;

public abstract class Controller {
    public enum ControlState {
        PERCENT_OUTPUT,
        POSITION,
        VELOCITY
    }

    public enum ControllerType {
        TalonFX,
        SparkMax,
        TalonSRX,
        VictorSPX,
        Simulation
    }

    protected ControlState controlState = ControlState.PERCENT_OUTPUT;
    protected RealControllerConstants constants;
    protected double goal = 0;
    private DigitalInput limitSwitch;
    private boolean preLimit = false;

    /**
     * Creates a new Ninjas controller
     *
     * @param constants the constants for the controller
     */
    public Controller(RealControllerConstants constants) {
        this.constants = constants;

        if(constants.isLimitSwitch && !constants.isVirtualLimit){
            limitSwitch = new DigitalInput(constants.limitSwitchID);
        }
    }

    /**
     * Sets percentage output to the controller
     *
     * @param percent how much to power the motor between -1 and 1
     * @see #setPosition(double)
     * @see #setVelocity(double)
     * @see #stop()
     */
    public void setPercent(double percent) {
        controlState = ControlState.PERCENT_OUTPUT;
    }

    /**
     * Sets position setpoint to the controller
     *
     * @param position the wanted position of the controller according to the encoder
     * @see #setPercent(double)
     * @see #setVelocity(double)
     * @see #stop()
     */
    public void setPosition(double position) {
        controlState = ControlState.POSITION;
        goal = position;
    }

    /**
     * Sets velocity setpoint output to the controller
     *
     * @param velocity the wanted velocity of the controller according to the encoder
     * @see #setPercent(double)
     * @see #setPosition(double)
     * @see #stop()
     */
    public void setVelocity(double velocity) {
        controlState = ControlState.VELOCITY;
        goal = velocity;
    }

    /**
     * Stops the controller of all movement
     *
     * @see #setPercent(double)
     * @see #setPosition(double)
     * @see #setVelocity(double)
     */
    public abstract void stop();

    /**
     * @return the rotational position of the motor
     */
    public abstract double getPosition();

    /**
     * @return the rotational velocity of the motor
     */
    public abstract double getVelocity();

    /**
     * @return the rotational acceleration of the motor
     */
    public abstract double getAcceleration();

    /**
     * @return the percent output of the controller
     */
    public abstract double getOutput();

    /**
     * @return the current the motor is taking
     */
    public abstract double getCurrent();

    /**
     * Sets the position in the encoder,so it thinks it is at that position
     *
     * @param position the position to set the encoder to
     */
    public abstract void setEncoder(double position);

    /**
     * Resets the encoder, sets it to the home position
     *
     * @see #isHomed
     */
    public void resetEncoder() {
        setEncoder(constants.homePosition);
    }

    /**
     * @return Whether the subsystem is homed: the encoder is at its home position
     * @see #resetEncoder
     */
    public boolean isHomed() {
        return Math.abs(constants.homePosition - getPosition()) < constants.positionGoalTolerance;
    }

    /**
     * @return goal/setpoint/reference of the controller, the target of PIDF / PID / Motion
     *     Magic...
     */
    public double getGoal() {
        return goal;
    }

    /**
     * @return whether the controller is at the goal, the target of PIDF / PID / Motion Magic...
     *     Will return false if not in position or velocity control
     */
    public boolean atGoal() {
        if (controlState == ControlState.POSITION)
            return Math.abs(getGoal() - getPosition()) < constants.positionGoalTolerance;
        else if (controlState == ControlState.VELOCITY)
            return Math.abs(getGoal() - getVelocity()) < constants.velocityGoalTolerance;

        return false;
    }

    /**
     * @return Whether the limit switch of the system is clicked now
     */
    public boolean getLimit() {
        if (!constants.isLimitSwitch)
            return false;

        if (Robot.isReal())
            return constants.isVirtualLimit
                ? (Math.abs(getCurrent() / RobotController.getBatteryVoltage()) > constants.virtualLimitStallThreshold && Math.signum(getOutput()) == constants.limitSwitchDirection) || (preLimit && Math.signum(getOutput()) != -constants.limitSwitchDirection)
                : constants.limitSwitchInverted != limitSwitch.get();
        else
            return Math.abs(constants.homePosition - getPosition()) < constants.positionGoalTolerance;
    }

    /** Runs controller periodic tasks, run it on the subsystem periodic */
    public void periodic() {
        if (constants.limitSwitchAutoStopReset && getLimit() && !preLimit)
            resetEncoder();
        if (constants.limitSwitchAutoStopReset && getLimit() && Math.signum(getOutput()) == constants.limitSwitchDirection)
            stop();
        preLimit = getLimit();
    }

    public static Controller createController(ControllerType type, ControllerConstants constants) {
        if (Robot.isReal()) {
            return switch (type) {
                case SparkMax -> new SparkMaxController(constants.real);
                case TalonSRX -> new TalonSRXController(constants.real);
                case VictorSPX -> new VictorSPXController(constants.real);
                default -> new TalonFXController(constants.real);
            };
        }

        return new SimulatedController(constants);
    }

    public static class ControllerIOInputs {
        public double Position;
        public double Velocity;
        public double Acceleration;
        public double Output;
        public double Current;
        public double Goal;
        public boolean LimitSwitch;
        public String ControlState;
        public String ControlType;
    }

    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Position = getPosition();
        inputs.Velocity = getVelocity();
        inputs.Acceleration = getAcceleration();
        inputs.Output = getOutput();
        inputs.Current = getCurrent();
        inputs.Goal = getGoal();
        inputs.LimitSwitch = getLimit();
        inputs.ControlState = controlState.toString();
        inputs.ControlType = constants.controlConstants.type == SmartControlType.NONE ? "N/A" : constants.controlConstants.type.toString();
    }
}
