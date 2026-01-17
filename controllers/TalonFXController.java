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

public class TalonFXController extends Controller {
    private final TalonFX main;
    private final TalonFX[] followers;

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
              .withStatorCurrentLimit(constants.base.currentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimit(constants.base.currentLimit)
              .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
              .withKP(constants.control.controlConstants.P)
              .withKI(constants.control.controlConstants.I)
              .withKD(constants.control.controlConstants.D)
              .withKS(constants.control.controlConstants.S)
              .withKV(constants.control.controlConstants.V)
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

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        main.set(percent);
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        switch (constants.control.controlConstants.type) {
            case PROFILED_PID, PROFILE:
                main.setControl(new MotionMagicVoltage(position));
                break;

            case PID:
                main.setControl(new PositionVoltage(position));
                break;

            case TORQUE_CURRENT:
                main.setControl(new PositionTorqueCurrentFOC(position));
                break;
        }
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        switch (constants.control.controlConstants.type) {
            case PROFILED_PID, PROFILE:
                main.setControl(new MotionMagicVelocityVoltage(velocity));
                break;

            case PID:
                main.setControl(new VelocityVoltage(velocity));
                break;

            case TORQUE_CURRENT:
                main.setControl(new VelocityTorqueCurrentFOC(velocity));
                break;
        }
    }

    @Override
    public void stop() {
        main.stopMotor();
    }

    @Override
    public double getPosition() {
        return main.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return main.getVelocity().getValueAsDouble();
    }

    @Override
    public double getAcceleration() {
        return main.getAcceleration().getValueAsDouble();
    }

    @Override
    public double getOutput() {
        return main.get();
    }

    @Override
    public double getCurrent() {
        return main.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setEncoder(double position) {
        main.setPosition(position);
    }

    public StatusSignal<Angle> getPositionSignal(int frequency) {
        StatusSignal<Angle> signal = main.getPosition();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    public StatusSignal<AngularVelocity> getVelocitySignal(int frequency) {
        StatusSignal<AngularVelocity> signal = main.getVelocity();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    public StatusSignal<AngularAcceleration> getAccelerationSignal(int frequency) {
        StatusSignal<AngularAcceleration> signal = main.getAcceleration();
        signal.setUpdateFrequency(frequency);
        return signal;
    }

    public StatusSignal<Current> getCurrentSignal(int frequency) {
        StatusSignal<Current> signal = main.getStatorCurrent();
        signal.setUpdateFrequency(frequency);
        return signal;
    }
}
