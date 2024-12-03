package com.ninjas4744.NinjasLib.Controllers;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;

public class NinjasTalonFXController extends NinjasController {
    private final TalonFX _main;
    private final TalonFX[] _followers;

    public NinjasTalonFXController(MainControllerConstants constants) {
        super(constants);

        _main = new TalonFX(constants.main.id);
        _main.getConfigurator()
          .apply(new TalonFXConfiguration()
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
              .withForwardSoftLimitEnable(constants.isMaxSoftLimit)
              .withReverseSoftLimitEnable(constants.isMinSoftLimit)
              .withForwardSoftLimitThreshold(constants.maxSoftLimit)
              .withReverseSoftLimitThreshold(constants.minSoftLimit))
            .withAudio(new AudioConfigs().withBeepOnBoot(true))
            .withMotorOutput(new MotorOutputConfigs()
              .withInverted(
                constants.main.inverted
                  ? InvertedValue.CounterClockwise_Positive
                  : InvertedValue.Clockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
              .withMotionMagicAcceleration(constants.controlConstants.kAcceleration)
              .withMotionMagicCruiseVelocity(constants.controlConstants.kCruiseVelocity))
            .withCurrentLimits(new CurrentLimitsConfigs()
              .withStatorCurrentLimit(constants.currentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimit(constants.currentLimit)
              .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
              .withKP(constants.controlConstants.kP)
              .withKI(constants.controlConstants.kI)
              .withKD(constants.controlConstants.kD)
              .withKS(constants.controlConstants.kS)
              .withKV(constants.controlConstants.kV))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(constants.encoderConversionFactor)));

        _followers = new TalonFX[constants.followers.length];
        for (int i = 0; i < _followers.length; i++) {
            _followers[i] = new TalonFX(constants.followers[i].id);
            _followers[i].getConfigurator().apply(new TalonFXConfiguration());
            _followers[i].setControl(new Follower(constants.main.id, constants.followers[i].inverted));
        }
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        _main.set(percent);
    }

    @Override
    public void setPosition(double position) {
        super.setPosition(position);

        switch (_constants.controlConstants.type) {
            case PROFILED_PID, PROFILE:
                _main.setControl(new MotionMagicVoltage(position));
                break;

            case PID:
                _main.setControl(new PositionVoltage(position));
                break;

            case TORQUE_CURRENT:
                _main.setControl(new PositionTorqueCurrentFOC(position));
                break;
        }
    }

    @Override
    public void setVelocity(double velocity) {
        super.setVelocity(velocity);

        switch (_constants.controlConstants.type) {
            case PROFILED_PID, PROFILE:
                _main.setControl(new MotionMagicVelocityVoltage(velocity));
                break;

            case PID:
                _main.setControl(new VelocityVoltage(velocity));
                break;

            case TORQUE_CURRENT:
                _main.setControl(new VelocityTorqueCurrentFOC(velocity));
                break;
        }
    }

    @Override
    public double getPosition() {
        return _main.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return _main.getVelocity().getValueAsDouble();
    }

    @Override
    public double getOutput() {
        return _main.get();
    }

    @Override
    public void setEncoder(double position) {
        _main.setPosition(position);
    }
}
