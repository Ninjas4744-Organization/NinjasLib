package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.NinjasLib.NinjasLogger;

public class Gyro {
    private GyroIO io;
    private GyroIO.GyroIOInputs inputs = new GyroIO.GyroIOInputs();

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    public Rotation2d getYawOffsetted() {
        return inputs.yawOffsetted;
    }

    public Rotation2d getPitch() {
        return inputs.pitch;
    }

    public Rotation2d getRoll() {
        return inputs.roll;
    }

    public double getAccelerationX() {
        return inputs.accelerationX;
    }

    public double getAccelerationY() {
        return inputs.accelerationY;
    }

    public double getAccelerationZ() {
        return inputs.accelerationZ;
    }

    public double[] getOdometryYawTimestamps() {
        return inputs.odometryYawTimestamps;
    }

    public Rotation2d[] getOdometryYawPositions() {
        return inputs.odometryYawPositions;
    }

    public void resetYaw(Rotation2d yaw) {
        io.resetGyroYaw(yaw);
    }

    public void periodic() {
        inputs = io.update();
        NinjasLogger.log("Swerve/Gyro/Yaw", inputs.yaw);
        NinjasLogger.log("Swerve/Gyro/Yaw Offsetted", inputs.yawOffsetted);
        NinjasLogger.log("Swerve/Gyro/Pitch", inputs.pitch);
        NinjasLogger.log("Swerve/Gyro/Roll", inputs.roll);
        NinjasLogger.log("Swerve/Gyro/Acceleration", new Translation3d(inputs.accelerationX, inputs.accelerationY, inputs.accelerationZ));
    }
}
