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

/**
 * Abstract base for a Ninjas motor controller wrapper: a hardware-agnostic API for driving a
 * single motor mechanism (with optional followers) in percent-output, position, or velocity
 * control, and for reading back its encoder position/velocity, current draw, and limit switches.
 * Concrete subclasses wrap a specific vendor API - {@link SparkMaxController} (REV SparkMax),
 * {@link TalonFXController} (CTRE TalonFX), {@link TalonSRXController} (CTRE TalonSRX),
 * {@link VictorSPXController} (CTRE VictorSPX) - or run entirely in software via
 * {@link SimulatedController}. Subsystems should generally obtain an instance through
 * {@link #createController(ControllerType, ControllerConstants)} rather than constructing one
 * directly, so the same subsystem code works both on the real robot and in simulation.
 */
public abstract class Controller {
    /** The closed/open-loop mode a {@link Controller} is currently commanded in. */
    public enum ControlState {
        PERCENT_OUTPUT,
        POSITION,
        VELOCITY
    }

    /**
     * The physical (or simulated) motor controller hardware a {@link Controller} wraps, used to
     * pick the right implementation in {@link #createController(ControllerType, ControllerConstants)}.
     */
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
     * Sets up the state shared by every {@link Controller} implementation: allocates a
     * {@link DigitalInput} for each non-virtual hard limit, and, if a CANcoder is configured to
     * run in {@link RealControllerConstants.CANCoder.CANCoderMode#Normal Normal} mode, constructs
     * and configures it. Subclasses call this via {@code super(constants)} before setting up
     * their own hardware.
     *
     * @param constants the controller configuration (base, control, soft/hard limits, CANcoder)
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
     * Switches to open-loop percent-output control. This base implementation only records the
     * new {@link ControlState}; every concrete subclass overrides it to also drive the motor
     * (calling {@code super.setPercent(percent)} first).
     *
     * @param percent how much to power the motor, between -1 and 1
     * @see #setPosition(double)
     * @see #setVelocity(double)
     * @see #stop()
     */
    public void setPercent(double percent) {
        controlState = ControlState.PERCENT_OUTPUT;
    }

    /**
     * Commands the controller to closed-loop position control. This is the main way to move a
     * mechanism to a specific setpoint (e.g. an elevator height or arm angle); the concrete
     * subclass drives the actual PID/Motion Magic/profile control per its
     * {@link ControllerConstants} once this base method records the goal.
     *
     * @param position the wanted position, in the units defined by the controller's gear
     *                  ratio/conversion configuration
     * @see #setPercent(double)
     * @see #setVelocity(double)
     * @see #stop()
     */
    public void setPosition(double position) {
        controlState = ControlState.POSITION;
        goal = position;
    }

    /**
     * Commands the controller to closed-loop velocity control, e.g. for a flywheel or drivetrain
     * wheel spun at a target speed. The concrete subclass drives the actual PID/feedforward once
     * this base method records the goal.
     *
     * @param velocity the wanted velocity, in the units defined by the controller's gear
     *                  ratio/conversion configuration, per second
     * @see #setPercent(double)
     * @see #setPosition(double)
     * @see #stop()
     */
    public void setVelocity(double velocity) {
        controlState = ControlState.VELOCITY;
        goal = velocity;
    }

    /**
     * Stops all motor movement by switching back to percent-output control at zero. Concrete
     * subclasses override this to also command the hardware to stop.
     *
     * @see #setPercent(double)
     * @see #setPosition(double)
     * @see #setVelocity(double)
     */
    public void stop() {
        controlState = ControlState.PERCENT_OUTPUT;
    }

    /**
     * The primary encoder reading every position-based subsystem call relies on. The value is in
     * the units defined by the concrete implementation's gear ratio/conversion configuration
     * (typically rotations of the mechanism, not the motor).
     *
     * @return the current position of the mechanism
     */
    public abstract double getPosition();

    /**
     * @return the rotational position of the absolute encoder, in rotations. Returns {@code 0} if
     * no CANcoder is configured, or it isn't running in
     * {@link RealControllerConstants.CANCoder.CANCoderMode#Normal Normal} mode.
     */
    public double getAbsolutePosition() {
        if (CANCoder != null)
            return CANCoder.getAbsolutePosition().getValue().in(Units.Rotations);
        return 0;
    }

    /**
     * The primary encoder-derived speed every velocity-based subsystem call relies on, in the
     * same units as {@link #getPosition()} per second.
     *
     * @return the current velocity of the mechanism
     */
    public abstract double getVelocity();

    /**
     * @return the current acceleration of the mechanism, in {@link #getVelocity()} units per second
     */
    public abstract double getAcceleration();

    /**
     * @return the applied motor output as a percentage, between -1 and 1
     */
    public abstract double getOutput();

    /**
     * @return the current drawn from the battery/CAN bus by the motor, in amps
     */
    public abstract double getSupplyCurrent();

    /**
     * @return the current flowing through the motor windings (stator current), in amps
     */
    public abstract double getStatorCurrent();

    /**
     * Overwrites the encoder's stored position without physically moving the mechanism - used to
     * (re)zero or home an encoder, e.g. when a limit switch triggers.
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
     * @param index the index of the limit in {@link RealControllerConstants.HardLimits#limits}
     * @return whether that limit switch of the system is clicked now (including virtual limits)
     */
    public boolean getLimit(int index) {
        if (index >= constants.hardLimits.limits.length)
            return false;

        if (Robot.isReal()) return constants.hardLimits.limits[index].isVirtual
                ? limitFrames[index] >= constants.hardLimits.limits[index].frames || (preLimits[index] && Math.signum(getVelocity()) != -constants.hardLimits.limits[index].direction)
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

    /**
     * Runs the controller's periodic bookkeeping - call this from the owning subsystem's
     * {@code periodic()} every loop. On a real robot this debounces each configured limit (real
     * switches by reading the {@link DigitalInput}, virtual limits by watching stator current
     * against {@link RealControllerConstants.HardLimits.HardLimit#virtualStallThreshold} while
     * within its position window) and, once a limit becomes newly active, invokes its
     * {@link RealControllerConstants.HardLimits.HardLimit#limitTriggerMethod} if enabled.
     */
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

            if (constants.hardLimits.limits[i].enableLimitTriggerMethod) {
                constants.hardLimits.limits[i].limitTriggerMethod.trigger(this, constants.hardLimits.limits[i], preLimits[i]);
            }

            preLimits[i] = getLimit(i);
        }
    }

    /**
     * Clears the "was previously at limit" flag for one virtual limit, e.g. after intentionally
     * driving off of it, so it doesn't immediately re-trigger.
     *
     * @param index the index of the limit in {@link RealControllerConstants.HardLimits#limits}
     */
    public void resetVirtualLimit(int index) {
        preLimits[index] = false;
    }

    /** Clears the "was previously at limit" flag for every configured limit. */
    public void resetVirtualLimits() {
        for (int i = 0; i < constants.hardLimits.limits.length; i++) {
            preLimits[i] = false;
        }
    }

    /**
     * Factory method that builds the right {@link Controller} implementation for the given
     * {@link ControllerType}: a real hardware wrapper when running on the robot
     * ({@link Robot#isReal()}), or a {@link SimulatedController} when running in simulation.
     * This is the preferred way to construct a controller so subsystem code doesn't need to
     * branch on real-vs-simulated itself.
     *
     * @param type      which hardware (or simulation) to create
     * @param constants the controller configuration
     * @return a new controller instance appropriate for the current robot mode
     */
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

    /**
     * Snapshots the controller's current state into a loggable, struct-serializable
     * {@link ControllerLogs} record (position, velocity, acceleration, output, currents, goal,
     * limit switches, etc.) - intended to be called once per loop and logged/published for
     * telemetry (e.g. via AdvantageKit or NetworkTables).
     *
     * @return a new {@link ControllerLogs} populated with this controller's current values
     */
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

    /**
     * A struct-serializable, loggable snapshot of a {@link Controller}'s state at one instant -
     * see {@link Controller#getLogs()}. Every field mirrors one of the controller's getters.
     */
    public static class ControllerLogs implements StructSerializable {
        /** The controller's position at the time of the snapshot; see {@link Controller#getPosition()}. */
        public double Position;
        /** The controller's velocity at the time of the snapshot; see {@link Controller#getVelocity()}. */
        public double Velocity;
        /** The controller's acceleration at the time of the snapshot; see {@link Controller#getAcceleration()}. */
        public double Acceleration;
        /** The controller's percent output at the time of the snapshot; see {@link Controller#getOutput()}. */
        public double Output;
        /** The controller's supply current, in amps; see {@link Controller#getSupplyCurrent()}. */
        public double SupplyCurrent;
        /** The controller's stator current, in amps; see {@link Controller#getStatorCurrent()}. */
        public double StatorCurrent;
        /** The controller's control goal at the time of the snapshot; see {@link Controller#getGoal()}. */
        public double Goal;
        /** Whether the controller was at its goal; see {@link Controller#atGoal()}. */
        public boolean AtGoal;
        /** Whether any limit switch was clicked; see {@link Controller#getLimit()}. */
        public boolean LimitSwitch;
        /** Per-index limit switch state (fixed size of 5); see {@link Controller#getLimit(int)}. */
        public boolean[] LimitSwitches = new boolean[5];
        /** The CANcoder absolute position, in rotations; see {@link Controller#getAbsolutePosition()}. */
        public double AbsolutePosition;
        /** The {@link Controller.ControlState} the controller was in, as a string. */
        public String ControlState = "";
        /** The configured {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType}, as a string, or {@code "N/A"} if none. */
        public String ControlType = "";

        /**
         * Constructs a fully-populated log snapshot. Prefer {@link Controller#getLogs()} over
         * calling this directly.
         *
         * @param position        see {@link #Position}
         * @param velocity        see {@link #Velocity}
         * @param acceleration    see {@link #Acceleration}
         * @param output          see {@link #Output}
         * @param supplyCurrent   see {@link #SupplyCurrent}
         * @param statorCurrent   see {@link #StatorCurrent}
         * @param goal            see {@link #Goal}
         * @param atGoal          see {@link #AtGoal}
         * @param limitSwitch     see {@link #LimitSwitch}
         * @param limitSwitches   see {@link #LimitSwitches}; {@code null} is replaced with a zeroed array of length 5
         * @param absolutePosition see {@link #AbsolutePosition}
         * @param controlState    see {@link #ControlState}
         * @param controlType     see {@link #ControlType}
         */
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

        /** Constructs a zeroed/empty log snapshot. */
        public ControllerLogs() {
            this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false, new boolean[5], 0.0, "", "");
        }

        /** The shared {@link Struct} implementation used to serialize/deserialize {@link ControllerLogs}, e.g. for NetworkTables/log replay. */
        public static final ControllerLogs.ControllerLogsStruct struct = new ControllerLogs.ControllerLogsStruct();

        /** WPILib {@link Struct} (de)serializer for {@link ControllerLogs}, enabling it to be logged and replayed as raw bytes. */
        public static class ControllerLogsStruct implements Struct<ControllerLogs> {

            /** @return {@link ControllerLogs}, the type this struct (de)serializes */
            @Override
            public Class<ControllerLogs> getTypeClass() {
                return ControllerLogs.class;
            }

            /** @return the struct's registered type name, {@code "ControllerLogs"} */
            @Override
            public String getTypeName() {
                return "ControllerLogs";
            }

            /** @return the fixed packed size in bytes of a serialized {@link ControllerLogs} */
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

            /** @return an empty array; {@link ControllerLogs} has no nested struct-typed fields */
            @Override
            public Struct<?>[] getNested() {
                // No nested structs (like Pose2d) are used in this class
                return new Struct<?>[]{};
            }

            /** @return the raw struct schema string describing {@link ControllerLogs}'s field layout */
            @Override
            public String getSchema() {
                return "double Position;double Velocity;double Acceleration;double Output;" +
                        "double SupplyCurrent;double StatorCurrent;double Goal;bool AtGoal;" +
                        "bool LimitSwitch;bool LimitSwitches[5];double AbsolutePosition;" +
                        "char ControlState[30];char ControlType[30]";
            }

            /**
             * Deserializes a {@link ControllerLogs} from its packed struct representation. The
             * {@code ControlState}/{@code ControlType} strings are read as fixed 30-byte fields
             * and trimmed of trailing padding.
             *
             * @param bb the buffer positioned at the start of a packed {@link ControllerLogs}
             * @return the deserialized {@link ControllerLogs}
             */
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

            /**
             * Serializes a {@link ControllerLogs} into its packed struct representation. The
             * {@code ControlState}/{@code ControlType} strings are written as fixed 30-byte
             * fields, truncated or zero-padded to fit; a {@code null} array or string is treated
             * as empty.
             *
             * @param bb    the buffer to write the packed bytes into
             * @param value the log snapshot to serialize
             */
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

            /** @return {@code false}; {@link ControllerLogs} instances are mutable */
            @Override
            public boolean isImmutable() {
                return false;
            }
        }
    }
}
