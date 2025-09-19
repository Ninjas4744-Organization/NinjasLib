package frc.lib.NinjasLib.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.NinjasLib.localization.OdometryThread;

import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;
    private boolean inverted;

    public GyroIOPigeon2(int id, boolean inverted, int frequency, String canbus) {
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

    @Override
    public void updateInputs(GyroIOInputsAutoLogged inputs) {
        BaseStatusSignal.refreshAll(yaw);
        inputs.Yaw = Rotation2d.fromRadians((inverted ? -1 : 1) * yaw.getValue().in(Units.Radians));
        inputs.Pitch = Rotation2d.fromRadians(pigeon.getPitch().getValue().in(Units.Radians));
        inputs.Roll = Rotation2d.fromRadians(pigeon.getRoll().getValue().in(Units.Radians));
//        inputs.AccelerationX = pigeon.getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationY = pigeon.getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationZ = pigeon.getAccelerationZ().getValue().in(Units.MetersPerSecondPerSecond);

        if (yawTimestampQueue != null) {
            inputs.odometryYawTimestamps =
                    yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryYawPositions =
                    yawPositionQueue.stream()
                            .map(x -> Rotation2d.fromDegrees((inverted ? -1 : 1) * x))
                            .toArray(Rotation2d[]::new);
            yawTimestampQueue.clear();
            yawPositionQueue.clear();
        }
    }

    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        System.out.print("Gyro: " + pigeon.getRotation2d().getDegrees() + " -> ");
        pigeon.setYaw(yaw.getDegrees(), 0);
        System.out.println(pigeon.getRotation2d().getDegrees());
    }
}
