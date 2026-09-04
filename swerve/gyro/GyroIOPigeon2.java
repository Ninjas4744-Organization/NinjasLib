package frc.lib.NinjasLib.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;

import java.util.Queue;

import static edu.wpi.first.units.Units.Radians;

/**
 * {@link GyroIO} implementation for a CTRE Pigeon 2 IMU over CAN. Registers its yaw signal with the
 * {@link OdometryThread} for high-frequency odometry sampling when {@code frequency} exceeds 50 Hz.
 */
public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;
    private boolean inverted;
    private Rotation2d yawOffset = Rotation2d.kZero;

    /**
     * @param id the Pigeon 2's CAN ID
     * @param inverted whether to negate the raw yaw reading
     * @param frequency desired yaw signal update frequency in Hz; if greater than 50, bus utilization
     *     is optimized and the yaw signal is registered with the {@link OdometryThread} for
     *     high-frequency odometry sampling
     * @param canbus the CAN bus the Pigeon 2 is connected to
     */
    public GyroIOPigeon2(int id, boolean inverted, int frequency, CANBus canbus) {
        pigeon = new Pigeon2(id, canbus);
        yaw = pigeon.getYaw();
        if (frequency > 50) {
            yaw.setUpdateFrequency(frequency);
            pigeon.optimizeBusUtilization();
            yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = OdometryThread.getInstance().registerSignal(yaw.clone());
        }
        this.inverted = inverted;
    }

    /**
     * {@inheritDoc}
     * <p>
     * Acceleration fields are currently left at zero (Pigeon 2 acceleration reads are disabled).
     */
    @Override
    public GyroIOInputs update() {
        GyroIOInputs inputs = new  GyroIOInputs();

        BaseStatusSignal.refreshAll(yaw);
        inputs.yaw = Rotation2d.fromRadians((inverted ? -1 : 1) * yaw.getValue().in(Units.Radians));
        inputs.yawOffsetted = Rotation2d.fromRadians((inverted ? -1 : 1) * yaw.getValue().in(Units.Radians)).plus(yawOffset);
        inputs.pitch = Rotation2d.fromRadians(pigeon.getPitch().getValue().in(Units.Radians));
        inputs.roll = Rotation2d.fromRadians(pigeon.getRoll().getValue().in(Units.Radians));
//        inputs.AccelerationX = pigeon.getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationY = pigeon.getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationZ = pigeon.getAccelerationZ().getValue().in(Units.MetersPerSecondPerSecond);

        if (yawTimestampQueue != null) {
            inputs.odometryYawTimestamps =
                    yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryYawPositions =
                    yawPositionQueue.stream()
                            .map(x -> Rotation2d.fromDegrees((inverted ? -1 : 1) * x).plus(yawOffset))
                            .toArray(Rotation2d[]::new);
            yawTimestampQueue.clear();
            yawPositionQueue.clear();
        }

        return inputs;
    }

    /** {@inheritDoc} Implemented by both re-seating the Pigeon's internal yaw and updating {@code yawOffset}. */
    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        NinjasLogger.logEvent("[Gyro Reset] " + (inverted ? -1 : 1) * this.yaw.getValue().in(Units.Degrees) + " -> " + yaw.getDegrees());

        if (inverted)
            yaw = yaw.unaryMinus();
        yawOffset = yawOffset.plus(Rotation2d.fromRadians(this.yaw.getValue().in(Radians)).minus(yaw));
        pigeon.setYaw(yaw.getDegrees(), 0);
    }
}
