package frc.lib.NinjasLib.controllers;

/**
 * The hardware-facing half of a {@link Controller}: the raw operations needed to drive a single
 * motor mechanism (with optional followers) and read its sensors. Implementations wrap a specific
 * vendor API - {@link SparkMaxIO} (REV SparkMax), {@link TalonFXIO} (CTRE TalonFX),
 * {@link TalonSRXIO} (CTRE TalonSRX), {@link VictorSPXIO} (CTRE VictorSPX) - or run entirely in
 * software via {@link SimulatedIO}. An implementation contains no limit logic and does not track
 * the commanded goal or {@link Controller.ControlState}; all of that lives in {@link Controller},
 * which owns exactly one {@code MotorIO}.
 */
public interface ControllerIO {
    /**
     * Drives the motor in open-loop percent output.
     *
     * @param percent how much to power the motor, between -1 and 1
     */
    void applyPercent(double percent);

    /**
     * Commands the motor's closed loop to the given position, per the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}.
     *
     * @param position the wanted position, in the units defined by the gear ratio/conversion configuration
     */
    void applyPosition(double position);

    /**
     * Commands the motor's closed loop to the given velocity, per the configured
     * {@link frc.lib.NinjasLib.controllers.constants.ControlConstants.ControlType ControlType}.
     *
     * @param velocity the wanted velocity, in the units defined by the gear ratio/conversion configuration, per second
     */
    void applyVelocity(double velocity);

    /** Stops all motor movement (including followers). */
    void stop();

    /**
     * @return the current position of the mechanism, in the units defined by the gear
     * ratio/conversion configuration (typically rotations of the mechanism, not the motor)
     */
    double getPosition();

    /** @return the current velocity of the mechanism, in {@link #getPosition()} units per second */
    double getVelocity();

    /** @return the current acceleration of the mechanism, in {@link #getVelocity()} units per second */
    double getAcceleration();

    /** @return the applied motor output as a percentage, between -1 and 1 */
    double getOutput();

    /** @return the current drawn from the battery/CAN bus by the motor, in amps */
    double getSupplyCurrent();

    /** @return the current flowing through the motor windings (stator current), in amps */
    double getStatorCurrent();

    /**
     * Overwrites the encoder's stored position without physically moving the mechanism.
     *
     * @param position the position to set the encoder to
     */
    void setEncoder(double position);

    /**
     * Per-loop hook for implementations that emulate closed-loop control in software (REV
     * SparkMax profiling, the simulated mechanism); called from {@link Controller#periodic()}
     * before the limits are processed. Since the implementation doesn't track the command itself,
     * the {@link Controller}'s current state is passed in. Does nothing by default, as for
     * hardware whose closed loop runs onboard.
     *
     * @param controlState the mode the {@link Controller} is currently commanded in
     * @param goal         the {@link Controller}'s current position/velocity goal
     */
    default void periodic(Controller.ControlState controlState, double goal) {}
}
