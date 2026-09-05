package frc.lib.NinjasLib.controllers.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.lib.NinjasLib.controllers.Controller;

/**
 * Real-hardware configuration for a {@link Controller}: motor IDs/inversion/current limits
 * ({@link #base}), control-loop gains and gear ratio ({@link #control}), software position limits
 * ({@link #softLimits}), limit-switch definitions ({@link #hardLimits}), and an optional CANcoder
 * ({@link #canCoder}).
 */
public class RealControllerConstants {
    /** Motor IDs, inversion, brake/coast mode, current limits, followers, and CAN bus. */
    public Base base = new Base();
    /** Closed-loop control gains, gear ratio/conversion, and goal tolerances. */
    public Control control = new Control();
    /** Software-enforced minimum/maximum position limits. */
    public SoftLimits softLimits = new SoftLimits();
    /** Hard (and virtual) limit switch definitions. */
    public HardLimits hardLimits = new HardLimits();
    /** Optional CANcoder configuration for absolute/remote position feedback. */
    public CANCoder canCoder = new CANCoder();

    /**
     * @param base the base motor/CAN configuration to apply
     * @return this instance, for chaining
     */
    public RealControllerConstants withBase(Base base) {
        this.base = base;
        return this;
    }

    /**
     * @param control the control-loop configuration to apply
     * @return this instance, for chaining
     */
    public RealControllerConstants withControl(Control control) {
        this.control = control;
        return this;
    }

    /**
     * @param softLimits the soft limits to apply
     * @return this instance, for chaining
     */
    public RealControllerConstants withSoftLimits(SoftLimits softLimits) {
        this.softLimits = softLimits;
        return this;
    }

    /**
     * @param hardLimits the hard/virtual limit switches to apply
     * @return this instance, for chaining
     */
    public RealControllerConstants withHardLimits(HardLimits.HardLimit[] hardLimits) {
        this.hardLimits.limits = hardLimits;
        return this;
    }

    /**
     * @param canCoder the CANcoder configuration to apply
     * @return this instance, for chaining
     */
    public RealControllerConstants withCANCoder(CANCoder canCoder) {
        this.canCoder = canCoder;
        return this;
    }

    /** Motor IDs, inversion, brake/coast mode, current limits, followers, and CAN bus for a controller. */
    public static class Base {
        /** Controller constants for the main controller in the subsystem */
        public SimpleControllerConstants main = new SimpleControllerConstants();

        /**
         * Whether the neutral mode of the controller should be brake or coast(in brake mode setting motor to 0 makes a sudden stop,
         * in coast mode setting motor to 0 makes it free and slowly climb down to zero due to friction).
         */
        public boolean isBrakeMode = true;

        /** Limit on current drawn from the battery/bus to the motor, in amps. */
        public double supplyCurrentLimit = 40;

        /** Limit on current flowing through the motor windings, in amps. */
        public double statorCurrentLimit = 80;

        /** Controller constants for the controllers that follow the main controller in the subsystem. */
        public SimpleControllerConstants[] followers = new SimpleControllerConstants[0];

        /** The name of the canbus the swerve is running on. 'rio' by default if CANivore is not present */
        public CANBus CANBus = com.ctre.phoenix6.CANBus.roboRIO();

        /**
         * @param main the main controller's ID/inversion
         * @return this instance, for chaining
         */
        public Base withMain(SimpleControllerConstants main) {
            this.main = main;
            return this;
        }

        /**
         * @param isBrakeMode whether the neutral mode should be brake ({@code true}) or coast ({@code false})
         * @return this instance, for chaining
         */
        public Base withIsBrakeMode(boolean isBrakeMode) {
            this.isBrakeMode = isBrakeMode;
            return this;
        }

        /**
         * @param supplyCurrentLimit the supply current limit, in amps
         * @return this instance, for chaining
         */
        public Base withSupplyCurrentLimit(double supplyCurrentLimit) {
            this.supplyCurrentLimit = supplyCurrentLimit;
            return this;
        }

        /**
         * @param statorCurrentLimit the stator current limit, in amps
         * @return this instance, for chaining
         */
        public Base withStatorCurrentLimit(double statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit;
            return this;
        }

        /**
         * @param followers the follower controllers' IDs/inversions
         * @return this instance, for chaining
         */
        public Base withFollowers(SimpleControllerConstants[] followers) {
            this.followers = followers;
            return this;
        }

        /**
         * @param CANBus the CAN bus the controller is on
         * @return this instance, for chaining
         */
        public Base withCANBus(CANBus CANBus) {
            this.CANBus = CANBus;
            return this;
        }

        /** The CAN ID and inversion setting for a single motor controller (main or follower). */
        public static class SimpleControllerConstants {
            /**
             * The ID of the controller, chosen in the device's configuration software like Phoenix Tuner X or Rev
             * Hardware Client
             */
            public int id = 0;

            /**
             * Whether to invert the output of this controller. If this controller is a follower it
             * will invert the main controller's output so if the main controller is inverted and this
             * follower is inverted it will be inverted twice so not inverted.
             */
            public boolean inverted = false;

            /** Constructs constants with the default ID ({@code 0}) and inversion ({@code false}). */
            public SimpleControllerConstants() {}

            /**
             * @param id       see {@link #id}
             * @param inverted see {@link #inverted}
             */
            public SimpleControllerConstants(int id, boolean inverted) {
                this.id = id;
                this.inverted = inverted;
            }

            /**
             * @param id the CAN ID to apply
             * @return this instance, for chaining
             */
            public SimpleControllerConstants withId(int id) {
                this.id = id;
                return this;
            }

            /**
             * @param inverted the inversion setting to apply
             * @return this instance, for chaining
             */
            public SimpleControllerConstants withInverted(boolean inverted) {
                this.inverted = inverted;
                return this;
            }
        }
    }

    /** Closed-loop control gains, gear ratio/conversion, and goal tolerances for a controller. */
    public static class Control {
        /** The PID/feedforward/profile gains and control scheme to use; see {@link ControlConstants}. */
        public ControlConstants controlConstants = new ControlConstants();

        /** The gear ratio between the motor and the subsystem including gears, pullies, gearboxes, etc. */
        public double gearRatio = 1;

        /**
         * Conversion between rotations of subsystem to whatever.
         * The calculation is: (rotations of motor) / gearRatio * conversionFactor.
         * So for example if I have an arm with gear ratio of 10, and I want it to be in degrees I would put 360.
         */
        public double conversionFactor = 1;

        /** The error which is considered atGoal(). if the error from the position goal is smaller than this value it will be considered atGoal(). */
        public double positionGoalTolerance = 0.05;

        /** The error which is considered atGoal(). if the error from the velocity goal is smaller than this value it will be considered atGoal(). */
        public double velocityGoalTolerance = 0.05;

        /** Whether to enable FOC. This only does something for TalonFX controllers. With FOC the torque is increased but the max speed is decreased from 100 to 95 rps */
        public boolean enableFOC = true;

        /**
         * @param controlConstants the PID/feedforward/profile gains to apply
         * @return this instance, for chaining
         */
        public Control withControlConstants(ControlConstants controlConstants) {
            this.controlConstants = controlConstants;
            return this;
        }

        /**
         * @param gearRatio        the motor-to-mechanism gear ratio
         * @param conversionFactor the mechanism-rotations-to-output-units conversion factor
         * @return this instance, for chaining
         */
        public Control withConversion(double gearRatio, double conversionFactor) {
            this.gearRatio = gearRatio;
            this.conversionFactor = conversionFactor;
            return this;
        }

        /**
         * @param positionGoalTolerance the position error below which {@code atGoal()} is {@code true}
         * @return this instance, for chaining
         */
        public Control withPositionGoalTolerance(double positionGoalTolerance) {
            this.positionGoalTolerance = positionGoalTolerance;
            return this;
        }

        /**
         * @param velocityGoalTolerance the velocity error below which {@code atGoal()} is {@code true}
         * @return this instance, for chaining
         */
        public Control withVelocityGoalTolerance(double velocityGoalTolerance) {
            this.velocityGoalTolerance = velocityGoalTolerance;
            return this;
        }

        /**
         * @param enableFOC whether to enable FOC (TalonFX only)
         * @return this instance, for chaining
         */
        public Control withEnableFOC(boolean enableFOC) {
            this.enableFOC = enableFOC;
            return this;
        }
    }

    /** Software-enforced minimum/maximum position limits for a controller, in mechanism units. */
    public static class SoftLimits {
        /** The down soft limit, makes the system unable to move under it */
        public double min = Double.NEGATIVE_INFINITY;

        /** The up soft limit, makes the system unable to move above it */
        public double max = Double.POSITIVE_INFINITY;

        /**
         * @param min the minimum position limit
         * @return this instance, for chaining
         */
        public SoftLimits withMin(double min) {
            this.min = min;
            return this;
        }

        /**
         * @param max the maximum position limit
         * @return this instance, for chaining
         */
        public SoftLimits withMax(double max) {
            this.max = max;
            return this;
        }
    }

    /** Container for a controller's hard (and virtual) limit switch definitions. */
    public static class HardLimits {
        /** The configured limit switches; indices here match the {@code index} argument of {@code Controller.getLimit(int)}. */
        public HardLimit[] limits = new HardLimit[0];

        /**
         * The configuration for one hard limit switch - either a real switch wired to a
         * {@link edu.wpi.first.wpilibj.DigitalInput}, or a "virtual" one inferred from motor
         * stall current. See {@link Controller#getLimit(int)} and {@link Controller#periodic()}.
         */
        public static class HardLimit {
            /** ID of limit switch used in the subsystem. */
            public int id = 0;

            /** Whether to use a virtual limit switch (according to the current the motor takes) instead of a real one. */
            public boolean isVirtual = false;

            /** How much current is needed to activate the virtual limit to behave like a real limit switch */
            public double virtualStallThreshold = 58;

            /** The minimum position of the encoder to apply the limit. If the encoder is under this value the limit will be ignored, so don't change this number for limits that reset the encoder */
            public double minPos = Double.NEGATIVE_INFINITY;

            /** The maximum position of the encoder to apply the limit. If the encoder is above this value the limit will be ignored, so don't change this number for limits that reset the encoder */
            public double maxPos = Double.POSITIVE_INFINITY;

            /** The amount of frames the limit needs to be turned on to count as clicked */
            public double frames = 1;

            /** Whether the limit switch is inverted. */
            public boolean inverted = false;

            /** the direction of movement in which the limit will be clicked, for example if an elevator goes down when given minus as output and the limit switch is at the bottom then this value should be -1. */
            public int direction = -1;

            /** Whether {@link #limitTriggerMethod} should be invoked when this limit newly becomes active. */
            public boolean enableLimitTriggerMethod = true;

            /**
             * Called from {@link Controller#periodic()} when this limit newly becomes active (and
             * {@link #enableLimitTriggerMethod} is {@code true}). By default it resets the encoder
             * to {@link #homePosition} and, if the motor is still being driven into the limit,
             * commands a position hold there - see {@link LimitTriggerMethod}.
             */
            public LimitTriggerMethod limitTriggerMethod = (controller, limitConstants, preLimit) -> {
                if (!preLimit)
                    controller.setEncoder(limitConstants.homePosition); // Reset encoder

                if (Math.signum(controller.getOutput()) == limitConstants.direction)
                    controller.setPosition(limitConstants.homePosition); // Set control to current position to hold position
            };

            /**
             * The home position of the subsystem where the limit switch is and is usually 0. when the limit
             * switch is hit the encoder will reset to this value.
             */
            public double homePosition = 0;

            /**
             * @param id the digital input ID of the limit switch
             * @return this instance, for chaining
             */
            public HardLimit withId(int id) {
                this.id = id;
                return this;
            }

            /**
             * @param isVirtual             whether this limit is inferred from stall current rather than a real switch
             * @param virtualStallThreshold the stall current threshold to activate the virtual limit
             * @return this instance, for chaining
             */
            public HardLimit withVirtual(boolean isVirtual, double virtualStallThreshold) {
                this.isVirtual = isVirtual;
                this.virtualStallThreshold = virtualStallThreshold;
                return this;
            }

            /**
             * @param minPos the minimum encoder position at which this limit is active
             * @return this instance, for chaining
             */
            public HardLimit withMinPos(double minPos) {
                this.minPos = minPos;
                return this;
            }

            /**
             * @param maxPos the maximum encoder position at which this limit is active
             * @return this instance, for chaining
             */
            public HardLimit withMaxPos(double maxPos) {
                this.maxPos = maxPos;
                return this;
            }

            /**
             * @param frames the number of consecutive frames the limit must read active to count as clicked
             * @return this instance, for chaining
             */
            public HardLimit withFrames(double frames) {
                this.frames = frames;
                return this;
            }

            /**
             * @param inverted whether the real limit switch's signal is inverted
             * @return this instance, for chaining
             */
            public HardLimit withInverted(boolean inverted) {
                this.inverted = inverted;
                return this;
            }

            /**
             * @param direction the direction of motion in which this limit is hit ({@code 1} or {@code -1})
             * @return this instance, for chaining
             */
            public HardLimit withDirection(int direction) {
                this.direction = direction;
                return this;
            }

            /**
             * @param enableLimitTriggerMethod whether to invoke {@link #limitTriggerMethod} when this limit triggers
             * @return this instance, for chaining
             */
            public HardLimit withEnableLimitTriggerMethod(boolean enableLimitTriggerMethod) {
                this.enableLimitTriggerMethod = enableLimitTriggerMethod;
                return this;
            }

            /**
             * @param limitTriggerMethod the callback to run when this limit newly becomes active
             * @return this instance, for chaining
             */
            public HardLimit withLimitTriggerMethod(LimitTriggerMethod limitTriggerMethod) {
                this.limitTriggerMethod = limitTriggerMethod;
                return this;
            }

            /**
             * @param homePosition the position the encoder is reset to when this limit triggers
             * @return this instance, for chaining
             */
            public HardLimit withHomePosition(double homePosition) {
                this.homePosition = homePosition;
                return this;
            }
        }

        /** Callback invoked by {@link Controller#periodic()} when a {@link HardLimit} newly becomes active. */
        @FunctionalInterface
        public interface LimitTriggerMethod {
            /**
             * @param controller     the controller whose limit triggered
             * @param limitConstants the configuration of the limit that triggered
             * @param preLimit       whether the limit was already active on the previous {@code periodic()} call
             */
            void trigger(Controller controller, HardLimit limitConstants, boolean preLimit);
        }
    }

    /** Configuration for an optional CTRE CANcoder providing absolute/remote position feedback. */
    public static class CANCoder {
        /** Whether a CANCoder is connected to the motor */
        public boolean enable = false;

        /** ID of the CANCoder in CAN */
        public int id = 0;

        /** Config of the CANCoder: offset and direction */
        public CANcoderConfiguration config = new CANcoderConfiguration();

        /** The mode of the CANCoder- Normal, Fused, Sync.
         * Normal: Enables the CANCoder through code and resets the encoder's position on command by robot code.
         * Fused: Resets position of encoder to CANCoder on motor startup, then there is no more connection to it.
         * Sync: Updates the encoder's position automatically to CANCoder every time it can. Through Phoenix Pro.
         */
        public CANCoderMode mode = CANCoderMode.Normal;

        /**
         * Sets the CANcoder's ID and enables it.
         *
         * @param id the CAN ID of the CANcoder
         * @return this instance, for chaining
         */
        public CANCoder withId(int id) {
            this.id = id;
            this.enable = true;
            return this;
        }

        /**
         * Sets the CANcoder's config (offset/direction) and enables it.
         *
         * @param config the CANcoder configuration to apply
         * @return this instance, for chaining
         */
        public CANCoder withConfig(CANcoderConfiguration config) {
            this.config = config;
            this.enable = true;
            return this;
        }

        /**
         * Sets the CANcoder's mode and enables it.
         *
         * @param mode see {@link #mode}
         * @return this instance, for chaining
         */
        public CANCoder withMode(CANCoderMode mode) {
            this.mode = mode;
            this.enable = true;
            return this;
        }

        /** How a configured CANcoder's absolute position is combined with the motor controller's own encoder; see {@link #mode}. */
        public enum CANCoderMode {
            Normal,
            Fused,
            Sync
        }
    }
}