package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.lib.NinjasLib.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import java.util.Queue;

public class GyroIOSim implements GyroIO {
    GyroSimulation gyro;
    private boolean inverted;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOSim(GyroSimulation gyro, boolean inverted) {
        this.gyro = gyro;
        this.inverted = inverted;
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(() -> gyro.getGyroReading().getDegrees());
    }

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

    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        NinjasLogger.logEvent("[Gyro Reset] " + (!inverted ? gyro.getGyroReading() : gyro.getGyroReading().unaryMinus()).getDegrees() + " -> " + yaw.getDegrees());

        if (inverted)
            yaw = yaw.unaryMinus();
        gyro.setRotation(yaw);
        System.out.println(gyro.getGyroReading().getDegrees());
    }
}
