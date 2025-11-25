package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public Rotation2d Yaw = Rotation2d.kZero;
        public Rotation2d YawOffsetted = Rotation2d.kZero;
        public Rotation2d Pitch = Rotation2d.kZero;
        public Rotation2d Roll = Rotation2d.kZero;
        public double AccelerationX;
        public double AccelerationY;
        public double AccelerationZ;

        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    default void updateInputs(GyroIOInputsAutoLogged inputs) {}

    default void resetGyroYaw(Rotation2d yaw) {}
}
