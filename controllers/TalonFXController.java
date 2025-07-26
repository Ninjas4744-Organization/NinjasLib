package frc.lib.NinjasLib.controllers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

        main = new TalonFX(constants.main.id);
        main.getConfigurator()
          .apply(new TalonFXConfiguration()
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(constants.maxSoftLimit != Double.MAX_VALUE)
                .withReverseSoftLimitEnable(constants.minSoftLimit != Double.MIN_VALUE)
                    .withForwardSoftLimitThreshold(constants.maxSoftLimit != Double.MAX_VALUE ? constants.maxSoftLimit : 0)
                    .withReverseSoftLimitThreshold(constants.minSoftLimit != Double.MIN_VALUE ? constants.minSoftLimit : 0))
                  .withAudio(new AudioConfigs().withBeepOnBoot(false))
            .withMotorOutput(new MotorOutputConfigs()
              .withInverted(
                constants.main.inverted
                  ? InvertedValue.CounterClockwise_Positive
                  : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(constants.isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(constants.controlConstants.acceleration)
                .withMotionMagicCruiseVelocity(constants.controlConstants.cruiseVelocity)
                .withMotionMagicJerk(constants.controlConstants.jerk))
            .withCurrentLimits(new CurrentLimitsConfigs()
              .withStatorCurrentLimit(constants.currentLimit)
              .withStatorCurrentLimitEnable(true)
              .withSupplyCurrentLimit(constants.currentLimit)
              .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
              .withKP(constants.controlConstants.P)
              .withKI(constants.controlConstants.I)
              .withKD(constants.controlConstants.D)
              .withKS(constants.controlConstants.S)
              .withKV(constants.controlConstants.V)
              .withKG(constants.controlConstants.G)
                    .withGravityType(constants.controlConstants.gravityType))
              .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(constants.gearRatio / constants.conversionFactor)));

        followers = new TalonFX[constants.followers.length];
        for (int i = 0; i < followers.length; i++) {
            followers[i] = new TalonFX(constants.followers[i].id);
            followers[i].getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withNeutralMode(constants.isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast));
            followers[i].setControl(new Follower(constants.main.id, constants.followers[i].inverted));
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

        switch (constants.controlConstants.type) {
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

        switch (constants.controlConstants.type) {
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

    public StatusSignal<Angle> getPositionSignal() {
        return main.getPosition();
    }

    public StatusSignal<AngularVelocity> getVelocitySignal() {
        return main.getVelocity();
    }

    public StatusSignal<AngularAcceleration> getAccelerationSignal() {
        return main.getAcceleration();
    }

    public StatusSignal<Current> getCurrentSignal() {
        return main.getStatorCurrent();
    }
}
