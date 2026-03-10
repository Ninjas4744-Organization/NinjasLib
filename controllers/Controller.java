package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.NinjasLib.controllers.constants.ControlConstants.SmartControlType;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLog;

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

    private DigitalInput[] limitSwitches;
    private boolean[] preLimits;
    private int[] virtualFrames;

    private CANcoder CANCoder;

    /**
     * Creates a new Ninjas controller
     *
     * @param constants the constants for the controller
     */
    public Controller(RealControllerConstants constants) {
        this.constants = constants;

        limitSwitches = new DigitalInput[constants.hardLimits.limits.length];
        preLimits = new boolean[constants.hardLimits.limits.length];
        virtualFrames = new int[constants.hardLimits.limits.length];

        for (int i = 0; i < constants.hardLimits.limits.length; i++){
            if (!constants.hardLimits.limits[i].isVirtual)
                limitSwitches[i] = new DigitalInput(constants.hardLimits.limits[i].id);
        }

        if (constants.canCoder.enable && constants.canCoder.mode == RealControllerConstants.CANCoder.CANCoderMode.Normal) {
            CANCoder = new CANcoder(constants.canCoder.id);
            CANCoder.getConfigurator().apply(constants.canCoder.config);
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
    public void stop() {
        controlState = ControlState.PERCENT_OUTPUT;
    }

    /**
     * @return the rotational position of the motor
     */
    public abstract double getPosition();

    /**
     * @return the rotational position of the absolute encoder in rotations. If there is no CANCoder, or it's not on normal mode, then will return 0.
     */
    public double getAbsolutePosition() {
        if (CANCoder != null)
            return CANCoder.getAbsolutePosition().getValue().in(Units.Rotations);
        return 0;
    }

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
    public abstract double getSupplyCurrent();


    public abstract double getStatorCurrent();

    /**
     * Sets the position in the encoder,so it thinks it is at that position
     *
     * @param position the position to set the encoder to
     */
    public abstract void setEncoder(double position);

    /**
     * @return Goal/Setpoint/Reference of the controller, the target of Profiled PID / PID / Motion Magic, etc...
     */
    public double getGoal() {
        return goal;
    }

    /**
     * @return Whether the controller is at its goal, the target of Profiled PID / PID / Motion Magic, etc... Will return false if not in position or velocity control
     */
    public boolean atGoal() {
        if (controlState == ControlState.POSITION)
            return Math.abs(getGoal() - getPosition()) < constants.control.positionGoalTolerance;
        else if (controlState == ControlState.VELOCITY)
            return Math.abs(getGoal() - getVelocity()) < constants.control.velocityGoalTolerance;

        return false;
    }

    /**
     * @return Whether a limit switch of the system is clicked now (including virtual)
     */
    public boolean getLimit(int index) {
        if (index >= constants.hardLimits.limits.length)
            return false;

        if (Robot.isReal()) return constants.hardLimits.limits[index].isVirtual
                ? virtualFrames[index] >= constants.hardLimits.limits[index].virtualFrames || (preLimits[index] && Math.signum(getOutput()) != -constants.hardLimits.limits[index].direction)
                : constants.hardLimits.limits[index].inverted != limitSwitches[index].get();
        else return Math.abs(constants.hardLimits.limits[index].homePosition - getPosition()) < constants.control.positionGoalTolerance;
    }

    /**
     * @return Whether any limit switch of the system is clicked now (including virtual)
     */
    public boolean getLimit() {
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            if (getLimit(i))
                return true;
        }

        return false;
    }

    /** Runs controller periodic tasks, run it on the subsystem periodic */
    public void periodic() {
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            if (Robot.isReal()) {
                if (constants.hardLimits.limits[i].isVirtual) {
                    if ((Math.abs(getStatorCurrent()) > constants.hardLimits.limits[i].virtualStallThreshold && Math.signum(getOutput()) == constants.hardLimits.limits[i].direction) && getPosition() >= constants.hardLimits.limits[i].virtualMinPos && getPosition() <= constants.hardLimits.limits[i].virtualMaxPos)
                        virtualFrames[i]++;
                    else
                        virtualFrames[i] = 0;
                }
            }

            if (constants.hardLimits.limits[i].autoStopReset && getLimit(i) && !preLimits[i])
                setEncoder(constants.hardLimits.limits[i].homePosition);
            if (constants.hardLimits.limits[i].autoStopReset && getLimit(i) && Math.signum(getOutput()) == constants.hardLimits.limits[i].direction)
                stop();

            preLimits[i] = getLimit(i);
        }
    }

    public void resetVirtualLimit(int index) {
        preLimits[index] = false;
    }

    public void resetVirtualLimits() {
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            preLimits[i] = false;
        }
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

    @AutoLog
    public static class ControllerIOInputs {
        public double Position;
        public double Velocity;
        public double Acceleration;
        public double Output;
        public double SupplyCurrent;
        public double StatorCurrent;
        public double Goal;
        public boolean AtGoal;
        public boolean LimitSwitch;
        public boolean[] LimitSwitches;
        public double AbsolutePosition;
        public String ControlState;
        public String ControlType;
    }

    public void updateInputs(ControllerIOInputs inputs) {
        inputs.Position = getPosition();
        inputs.Velocity = getVelocity();
        inputs.Acceleration = getAcceleration();
        inputs.Output = getOutput();
        inputs.SupplyCurrent = getSupplyCurrent();
        inputs.StatorCurrent = getStatorCurrent();
        inputs.Goal = getGoal();
        inputs.AtGoal = atGoal();
        inputs.LimitSwitch = getLimit();

        inputs.LimitSwitches = new boolean[constants.hardLimits.limits.length];
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            inputs.LimitSwitches[i] = getLimit(i);
        }

        inputs.AbsolutePosition = getAbsolutePosition();
        inputs.ControlState = controlState.toString();
        inputs.ControlType = constants.control.controlConstants.type == SmartControlType.NONE ? "N/A" : constants.control.controlConstants.type.toString();
    }
}
