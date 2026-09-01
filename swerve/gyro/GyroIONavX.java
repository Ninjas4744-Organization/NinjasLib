package frc.lib.NinjasLib.swerve.gyro;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.NinjasLogger;
import frc.lib.NinjasLib.localization.OdometryThread;

import java.util.Queue;

public class GyroIONavX implements GyroIO{
    private AHRS navX;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private boolean inverted;
    private Rotation2d yawOffset = Rotation2d.kZero;

    public GyroIONavX(int frequency, boolean inverted) {
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI, frequency);
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(navX::getYaw);
        this.inverted = inverted;
    }

    @Override
    public GyroIOInputs update() {
        GyroIOInputs inputs = new GyroIOInputs();

        inputs.yaw = Rotation2d.fromDegrees((inverted ? -1 : 1) * navX.getYaw());
        inputs.yawOffsetted = Rotation2d.fromDegrees((inverted ? -1 : 1) * navX.getYaw()).plus(yawOffset);
        inputs.pitch = Rotation2d.fromDegrees(navX.getPitch());
        inputs.roll = Rotation2d.fromDegrees(navX.getRoll());
        inputs.accelerationX = navX.getWorldLinearAccelX() * 9.81;
        inputs.accelerationY = navX.getWorldLinearAccelY() * 9.81;
        inputs.accelerationZ = navX.getWorldLinearAccelZ() * 9.81;

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
                yawPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromDegrees((inverted ? -1 : 1) * value).plus(yawOffset))
                        .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();

        return inputs;
    }

    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        NinjasLogger.logEvent("[Gyro Reset] " + (inverted ? -1 : 1) * navX.getYaw() + " -> " + yaw.getDegrees());

        if (inverted)
            yaw = yaw.unaryMinus();
        yawOffset = yawOffset.plus(Rotation2d.fromDegrees(navX.getYaw()).minus(yaw));
        navX.reset();
        navX.setAngleAdjustment(yaw.getDegrees());
    }
}
