package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;

/**
 * {@link Controller} implementation that wraps a CTRE TalonFX (Falcon 500/Kraken X60) motor
 * controller via the Phoenix 6 API. Unlike the other real controller wrappers, closed-loop
 * control (PID, Motion Magic, torque-current FOC) runs entirely onboard the TalonFX firmware, so
 * this class does not override {@link Controller#periodic()}.
 */
public class TalonFXController extends Controller {
    private final TalonFX main;
    private final TalonFX[] followers;

    /**
     * Constructs and applies a full {@link TalonFXConfiguration} to the main TalonFX - soft
     * limits, motor output inversion/neutral mode, Motion Magic constraints, stator/supply
     * current limits, Slot0 PIDF/gravity gains, and the feedback source (either the internal
     * rotor, scaled by the gear ratio/conversion factor, or a remote CANcoder fused/synced per
     * {@link RealControllerConstants.CANCoder#mode}) - then constructs each follower TalonFX to
     * follow it.
     *
     * @param constants the controller configuration
     */
    public TalonFXController(RealControllerConstants constants) {
        super(constants);

        main = new TalonFX(constants.base.main.id, constants.base.CANBus);
        main.getConfigurator()
          .apply(new TalonFXConfiguration()
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(constants.softLimits.max != Double.POSITIVE_INFINITY)
                .withReverseSoftLimitEnable(constants.softLimits.min != Double.NEGATIVE_INFINITY)
                    .withForwardSoftLimitThreshold(constants.softLimits.max != Double.POSITIVE_INFINITY ? constants.softLimits.max : 0)
                    .withReverseSoftLimitThreshold(constants.softLimits.min != Double.NEGATIVE_INFINITY ? constants.softLimits.min : 0))
                  .withAudio(new AudioConfigs().withBeepOnBoot(false))
            .withMotorOutput(new MotorOutputConfigs()
              .withInverted(
                constants.base.main.inverted
                  ? InvertedValue.CounterClockwise_Positive
                  : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(constants.base.isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(constants.control.controlConstants.acceleration)
                .withMotionMagicCruiseVelocity(constants.control.controlConstants.cruiseVelocity)
                .withMotionMagicJerk(constants.control.controlConstants.jerk))
            .withCurrentLimits(new CurrentLimitsConfigs()
              .withStatorCurrentLimit(constants.base.statorCurrentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimit(constants.base.supplyCurrentLimit)
              .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
              .withKP(constants.control.controlConstants.P)
              .withKI(constants.control.controlConstants.I)
              .withKD(constants.control.controlConstants.D)
              .withKS(constants.control.controlConstants.S)
              .withKV(constants.control.controlConstants.V)
              .withKA(constants.control.controlConstants.A)
              .withKG(constants.control.controlConstants.G)
                    .withGravityType(constants.control.controlConstants.gravityType))
              .withFeedback(constants.canCoder.enable && constants.canCoder.mode != RealControllerConstants.CANCoder.CANCoderMode.Normal
                    ? new FeedbackConfigs().withFeedbackRemoteSensorID(constants.canCoder.id).withFeedbackSensorSource(constants.canCoder.mode == RealControllerConstants.CANCoder.CANCoderMode.Fused ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.SyncCANcoder)
                    : new FeedbackConfigs().withSensorToMechanismRatio(constants.control.gearRatio / constants.control.conversionFactor)));

        followers = new TalonFX[constants.base.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonFX(constants.base.followers[i].id, constants.base.CANBus);
            followers[i].getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(constants.base.isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast));
            followers[i].setControl(new Follower(constants.base.main.id, constants.base.followers[i].inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }
    }

    /**
     * Commands the main TalonFX with a {@link DutyCycleOut} request.
     *
     * @param percent how much to power the motor, between -1 and 1
     */
    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        main.setControl(new DutyCycleOut(percent).withEnableFOC(constants.control.enableFOC));
    }

    /**
     * Commands the main TalonFX's onboard closed loop to the given position, using the control
     * request that matches the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}:
     * Motion Magic for profiled/{@code PROFILE} control, plain position PID for {@code PIDF}, or
     * position torque-current FOC for {@code TORQUE_CURRENT}.
     *
     * @param position the wanted position
     */
    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        switch (constants.control.controlConstants.type) {
            case PROFILED_PIDF, PROFILE:
                main.setControl(new MotionMagicVoltage(position).withEnableFOC(constants.control.enableFOC));
                break;

            case PIDF:
                main.setControl(new PositionVoltage(position).withEnableFOC(constants.control.enableFOC));
                break;

            case TORQUE_CURRENT:
                main.setControl(new PositionTorqueCurrentFOC(position));
                break;
        }
    }

    /**
     * Commands the main TalonFX's onboard closed loop to the given velocity, using the control
     * request that matches the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}:
     * Motion Magic velocity for profiled/{@code PROFILE} control, plain velocity PID for
     * {@code PIDF}, or velocity torque-current FOC for {@code TORQUE_CURRENT}.
     *
     * @param velocity the wanted velocity
     */
    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        switch (constants.control.controlConstants.type) {
            case PROFILED_PIDF, PROFILE:
                main.setControl(new MotionMagicVelocityVoltage(velocity).withEnableFOC(constants.control.enableFOC));
                break;

            case PIDF:
                main.setControl(new VelocityVoltage(velocity).withEnableFOC(constants.control.enableFOC));
                break;

            case TORQUE_CURRENT:
                main.setControl(new VelocityTorqueCurrentFOC(velocity));
                break;
        }
    }

    /** Stops the main TalonFX (and its followers) via {@link TalonFX#stopMotor()}. */
    @Override
    public void stop() {
        super.stop();
        main.stopMotor();
    }

    /** @return the main TalonFX's position signal (rotations, scaled by the configured feedback/sensor ratio) */
    @Override
    public double getPosition() {
        return main.getPosition().getValueAsDouble();
    }

    /** @return the main TalonFX's velocity signal (rotations per second) */
    @Override
    public double getVelocity() {
        return main.getVelocity().getValueAsDouble();
    }

    /** @return the main TalonFX's acceleration signal (rotations per second squared) */
    @Override
    public double getAcceleration() {
        return main.getAcceleration().getValueAsDouble();
    }

    /** @return the main TalonFX's applied duty cycle output, between -1 and 1 */
    @Override
    public double getOutput() {
        return main.get();
    }

    /** @return the main TalonFX's supply current signal, in amps */
    @Override
    public double getSupplyCurrent() {
        return main.getSupplyCurrent().getValueAsDouble();
    }

    /** @return the main TalonFX's stator current signal, in amps */
    public double getStatorCurrent() {
        return main.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Overwrites the main TalonFX's position signal.
     *
     * @param position the position to set the encoder to
     */
    @Override
    public void setEncoder(double position) {
        main.setPosition(position);
    }

    /**
     * Escape hatch for accessing the underlying Phoenix 6 {@link TalonFX} directly for anything
     * not exposed through the {@link Controller} API.
     *
     * @return the main TalonFX device
     */
    public TalonFX getController() {
        return main;
    }

    /**
     * Returns the main TalonFX's position {@link StatusSignal} with its CAN update frequency set,
     * for callers (e.g. swerve odometry) that need to fetch and timestamp several signals
     * together via {@link com.ctre.phoenix6.BaseStatusSignal#waitForAll}.
     *
     * @param frequency the desired update frequency, in Hz
     * @return the position status signal
     */
    public StatusSignal<Angle> getPositionSignal(int frequency) {
        StatusSignal<Angle> signal = main.getPosition();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    /**
     * Returns the main TalonFX's velocity {@link StatusSignal} with its CAN update frequency set,
     * for callers (e.g. swerve odometry) that need to fetch and timestamp several signals
     * together via {@link com.ctre.phoenix6.BaseStatusSignal#waitForAll}.
     *
     * @param frequency the desired update frequency, in Hz
     * @return the velocity status signal
     */
    public StatusSignal<AngularVelocity> getVelocitySignal(int frequency) {
        StatusSignal<AngularVelocity> signal = main.getVelocity();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    /**
     * Returns the main TalonFX's acceleration {@link StatusSignal} with its CAN update frequency
     * set, for callers that need to fetch and timestamp several signals together via
     * {@link com.ctre.phoenix6.BaseStatusSignal#waitForAll}.
     *
     * @param frequency the desired update frequency, in Hz
     * @return the acceleration status signal
     */
    public StatusSignal<AngularAcceleration> getAccelerationSignal(int frequency) {
        StatusSignal<AngularAcceleration> signal = main.getAcceleration();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    /**
     * Returns the main TalonFX's stator current {@link StatusSignal} with its CAN update
     * frequency set, for callers that need to fetch and timestamp several signals together via
     * {@link com.ctre.phoenix6.BaseStatusSignal#waitForAll}.
     *
     * @param frequency the desired update frequency, in Hz
     * @return the stator current status signal
     */
    public StatusSignal<Current> getCurrentSignal(int frequency) {
        StatusSignal<Current> signal = main.getStatorCurrent();
        signal.setUpdateFrequency(frequency);
        return signal;
    }
}
