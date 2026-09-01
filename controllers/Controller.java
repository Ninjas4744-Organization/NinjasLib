package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.robot.Robot;

import java.nio.ByteBuffer;

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
    private int[] limitFrames;

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
        limitFrames = new int[constants.hardLimits.limits.length];

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
                ? limitFrames[index] >= constants.hardLimits.limits[index].frames || (preLimits[index] && Math.signum(getOutput()) != -constants.hardLimits.limits[index].direction)
                : limitFrames[index] >= constants.hardLimits.limits[index].frames;
        else return constants.hardLimits.limits[index].direction > 0 ? getPosition() >= constants.hardLimits.limits[index].homePosition : getPosition() <= constants.hardLimits.limits[index].homePosition;
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
                    if ((Math.abs(getStatorCurrent()) > constants.hardLimits.limits[i].virtualStallThreshold && Math.signum(getOutput()) == constants.hardLimits.limits[i].direction) && getPosition() >= constants.hardLimits.limits[i].minPos && getPosition() <= constants.hardLimits.limits[i].maxPos)
                        limitFrames[i]++;
                    else
                        limitFrames[i] = 0;
                } else {
                    if (constants.hardLimits.limits[i].inverted != limitSwitches[i].get())
                        limitFrames[i]++;
                    else
                        limitFrames[i] = 0;
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

    public ControllerLogs getLogs() {
        ControllerLogs logs = new ControllerLogs();

        logs.Position = getPosition();
        logs.Velocity = getVelocity();
        logs.Acceleration = getAcceleration();
        logs.Output = getOutput();
        logs.SupplyCurrent = getSupplyCurrent();
        logs.StatorCurrent = getStatorCurrent();
        logs.Goal = getGoal();
        logs.AtGoal = atGoal();
        logs.LimitSwitch = getLimit();

        logs.LimitSwitches = new boolean[constants.hardLimits.limits.length];
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            logs.LimitSwitches[i] = getLimit(i);
        }

        logs.AbsolutePosition = getAbsolutePosition();
        logs.ControlState = controlState.toString();
        logs.ControlType = constants.control.controlConstants.type == ControlType.NONE ? "N/A" : constants.control.controlConstants.type.toString();

        return logs;
    }

    public static class ControllerLogs implements StructSerializable {
        public double Position;
        public double Velocity;
        public double Acceleration;
        public double Output;
        public double SupplyCurrent;
        public double StatorCurrent;
        public double Goal;
        public boolean AtGoal;
        public boolean LimitSwitch;
        public boolean[] LimitSwitches = new boolean[5];
        public double AbsolutePosition;
        public String ControlState = "";
        public String ControlType = "";

        public ControllerLogs(double position, double velocity, double acceleration, double output,
                              double supplyCurrent, double statorCurrent, double goal, boolean atGoal,
                              boolean limitSwitch, boolean[] limitSwitches, double absolutePosition,
                              String controlState, String controlType) {
            this.Position = position;
            this.Velocity = velocity;
            this.Acceleration = acceleration;
            this.Output = output;
            this.SupplyCurrent = supplyCurrent;
            this.StatorCurrent = statorCurrent;
            this.Goal = goal;
            this.AtGoal = atGoal;
            this.LimitSwitch = limitSwitch;
            this.LimitSwitches = limitSwitches != null ? limitSwitches : new boolean[5];
            this.AbsolutePosition = absolutePosition;
            this.ControlState = controlState;
            this.ControlType = controlType;
        }

        public ControllerLogs() {
            this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false, new boolean[5], 0.0, "", "");
        }

        public static final ControllerLogs.ControllerLogsStruct struct = new ControllerLogs.ControllerLogsStruct();

        public static class ControllerLogsStruct implements Struct<ControllerLogs> {

            @Override
            public Class<ControllerLogs> getTypeClass() {
                return ControllerLogs.class;
            }

            @Override
            public String getTypeName() {
                return "ControllerLogs";
            }

            @Override
            public int getSize() {
                int size = 0;
                size += kSizeDouble; // Position
                size += kSizeDouble; // Velocity
                size += kSizeDouble; // Acceleration
                size += kSizeDouble; // Output
                size += kSizeDouble; // SupplyCurrent
                size += kSizeDouble; // StatorCurrent
                size += kSizeDouble; // Goal
                size += kSizeBool; // AtGoal
                size += kSizeBool; // LimitSwitch
                size += 5 * kSizeBool; // LimitSwitches (Only 5)
                size += kSizeDouble; // AbsolutePosition
                size += 30; // ControlState (30 chars)
                size += 30; // ControlType (30 chars)
                return size;
            }

            @Override
            public Struct<?>[] getNested() {
                // No nested structs (like Pose2d) are used in this class
                return new Struct<?>[]{};
            }

            @Override
            public String getSchema() {
                return "double Position;double Velocity;double Acceleration;double Output;" +
                        "double SupplyCurrent;double StatorCurrent;double Goal;bool AtGoal;" +
                        "bool LimitSwitch;bool LimitSwitches[5];double AbsolutePosition;" +
                        "char ControlState[30];char ControlType[30]";
            }

            @Override
            public ControllerLogs unpack(ByteBuffer bb) {
                ControllerLogs logs = new ControllerLogs();

                logs.Position = bb.getDouble();
                logs.Velocity = bb.getDouble();
                logs.Acceleration = bb.getDouble();
                logs.Output = bb.getDouble();
                logs.SupplyCurrent = bb.getDouble();
                logs.StatorCurrent = bb.getDouble();
                logs.Goal = bb.getDouble();
                logs.AtGoal = bb.get() != 0;
                logs.LimitSwitch = bb.get() != 0;

                logs.LimitSwitches = new boolean[5];
                for (int i = 0; i < 5; i++) {
                    logs.LimitSwitches[i] = bb.get() != 0;
                }

                logs.AbsolutePosition = bb.getDouble();

                // Unpack 30-byte String for ControlState
                byte[] controlStateBytes = new byte[30];
                bb.get(controlStateBytes);
                logs.ControlState = new String(controlStateBytes).trim();

                // Unpack 30-byte String for ControlType
                byte[] controlTypeBytes = new byte[30];
                bb.get(controlTypeBytes);
                logs.ControlType = new String(controlTypeBytes).trim();

                return logs;
            }

            @Override
            public void pack(ByteBuffer bb, ControllerLogs value) {
                bb.putDouble(value.Position);
                bb.putDouble(value.Velocity);
                bb.putDouble(value.Acceleration);
                bb.putDouble(value.Output);
                bb.putDouble(value.SupplyCurrent);
                bb.putDouble(value.StatorCurrent);
                bb.putDouble(value.Goal);
                bb.put((byte) (value.AtGoal ? 1 : 0));
                bb.put((byte) (value.LimitSwitch ? 1 : 0));

                // Pack boolean array safely up to 5 elements
                for (int i = 0; i < 5; i++) {
                    boolean val = (value.LimitSwitches != null && i < value.LimitSwitches.length)
                            ? value.LimitSwitches[i] : false;
                    bb.put((byte) (val ? 1 : 0));
                }

                bb.putDouble(value.AbsolutePosition);

                // Pack String as exactly 30 bytes
                byte[] stateBytes = value.ControlState != null ? value.ControlState.getBytes() : new byte[0];
                for (int i = 0; i < 30; i++) {
                    bb.put((byte) (i < stateBytes.length ? stateBytes[i] : 0));
                }

                // Pack String as exactly 30 bytes
                byte[] typeBytes = value.ControlType != null ? value.ControlType.getBytes() : new byte[0];
                for (int i = 0; i < 30; i++) {
                    bb.put((byte) (i < typeBytes.length ? typeBytes[i] : 0));
                }
            }

            @Override
            public boolean isImmutable() {
                return false;
            }
        }
    }
}
