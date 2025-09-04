package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public void updateInputs(GyroIOInputsAutoLogged inputs) {
        inputs.Yaw = !inverted ? gyro.getGyroReading() : gyro.getGyroReading().unaryMinus();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream()
                        .map(x -> Rotation2d.fromDegrees((inverted ? -1 : 1) * x))
                        .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        System.out.print("Gyro: " + gyro.getGyroReading().getDegrees() + " -> ");
        gyro.setRotation(yaw);
        System.out.println(gyro.getGyroReading().getDegrees());
    }
}
