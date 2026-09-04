package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import java.util.Queue;

/**
 * Simulation implementation of {@link GyroIO}, backed by an ironmaple {@link GyroSimulation} instead
 * of real hardware. Unlike the real implementations, gyro resets here do not track a separate offset;
 * they set the simulated rotation directly.
 */
public class GyroIOSim implements GyroIO {
    GyroSimulation gyro;
    private boolean inverted;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    /**
     * @param gyro the underlying simulated gyro to read from
     * @param inverted whether to negate the simulated yaw reading
     */
    public GyroIOSim(GyroSimulation gyro, boolean inverted) {
        this.gyro = gyro;
        this.inverted = inverted;
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(() -> gyro.getGyroReading().getDegrees());
    }

    /** {@inheritDoc} Note that pitch, roll and acceleration are not simulated and stay at their default values. */
    @Override
    public GyroIOInputs update() {
        GyroIOInputs inputs = new GyroIOInputs();

        inputs.yaw = !inverted ? gyro.getGyroReading() : gyro.getGyroReading().unaryMinus();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream()
                        .map(x -> Rotation2d.fromDegrees((inverted ? -1 : 1) * x))
                        .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();

        return inputs;
    }

    /** {@inheritDoc} Implemented by setting the simulated gyro's rotation directly. */
    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        NinjasLogger.logEvent("[Gyro Reset] " + (!inverted ? gyro.getGyroReading() : gyro.getGyroReading().unaryMinus()).getDegrees() + " -> " + yaw.getDegrees());

        if (inverted)
            yaw = yaw.unaryMinus();
        gyro.setRotation(yaw);
        System.out.println(gyro.getGyroReading().getDegrees());
    }
}
