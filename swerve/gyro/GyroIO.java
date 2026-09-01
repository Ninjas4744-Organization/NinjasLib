package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    class GyroIOInputs {
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d yawOffsetted = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;
        public Rotation2d roll = Rotation2d.kZero;
        public double accelerationX;
        public double accelerationY;
        public double accelerationZ;

        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    GyroIOInputs update();
    void resetGyroYaw(Rotation2d yaw);
}
