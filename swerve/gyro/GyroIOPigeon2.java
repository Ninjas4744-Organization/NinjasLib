package frc.lib.NinjasLib.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.NinjasLib.localization.OdometryThread;

import java.util.Queue;

import static edu.wpi.first.units.Units.Radians;

public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;
    private boolean inverted;
    private Rotation2d yawOffset = Rotation2d.kZero;

    private final boolean useOdometryThread;

    // Cached arrays to avoid GC pressure from stream operations
    private double[] yawTimestampArray = new double[0];
    private Rotation2d[] yawPositionArray = new Rotation2d[0];

    public GyroIOPigeon2(int id, boolean inverted, int frequency, CANBus canbus) {
        pigeon = new Pigeon2(id, canbus);
        yaw = pigeon.getYaw();
        useOdometryThread = frequency > 50;
        if (useOdometryThread) {
            yaw.setUpdateFrequency(frequency);
            pigeon.optimizeBusUtilization();
            yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue = OdometryThread.getInstance().registerSignal(yaw.clone());
        }
        this.inverted = inverted;
    }

    @Override
    public void updateInputs(GyroIOInputsAutoLogged inputs) {
        // Only refresh from CAN when odometry thread isn't already refreshing the signal
        if (!useOdometryThread) {
            BaseStatusSignal.refreshAll(yaw);
        }
        inputs.Yaw = Rotation2d.fromRadians((inverted ? -1 : 1) * yaw.getValue().in(Units.Radians));
        inputs.YawOffsetted = Rotation2d.fromRadians((inverted ? -1 : 1) * yaw.getValue().in(Units.Radians)).plus(yawOffset);
        inputs.Pitch = Rotation2d.fromRadians(pigeon.getPitch().getValue().in(Units.Radians));
        inputs.Roll = Rotation2d.fromRadians(pigeon.getRoll().getValue().in(Units.Radians));
//        inputs.AccelerationX = pigeon.getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationY = pigeon.getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.AccelerationZ = pigeon.getAccelerationZ().getValue().in(Units.MetersPerSecondPerSecond);

        if (yawTimestampQueue != null) {
            int size = yawTimestampQueue.size();

            if (yawTimestampArray.length != size) yawTimestampArray = new double[size];
            if (yawPositionArray.length != size) yawPositionArray = new Rotation2d[size];

            int idx = 0;
            for (Double val : yawTimestampQueue) yawTimestampArray[idx++] = val;

            idx = 0;
            for (Double val : yawPositionQueue) yawPositionArray[idx++] = Rotation2d.fromDegrees((inverted ? -1 : 1) * val).plus(yawOffset);

            inputs.odometryYawTimestamps = yawTimestampArray;
            inputs.odometryYawPositions = yawPositionArray;

            yawTimestampQueue.clear();
            yawPositionQueue.clear();
        }
    }

    @Override
    public void resetGyroYaw(Rotation2d yaw) {
        System.out.print("Gyro: " + pigeon.getRotation2d().getDegrees() + " -> " + yaw.getDegrees());
        yawOffset = yawOffset.plus(Rotation2d.fromRadians(this.yaw.getValue().in(Radians)).minus(yaw));
        pigeon.setYaw(yaw.getDegrees(), 0);
    }
}
