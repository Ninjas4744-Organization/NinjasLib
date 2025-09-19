package frc.lib.NinjasLib.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Gyro {
    private GyroIO io;
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public Rotation2d getYaw() {
        return inputs.Yaw;
    }

    public Rotation2d getPitch() {
        return inputs.Pitch;
    }

    public Rotation2d getRoll() {
        return inputs.Roll;
    }

    public double getAccelerationX() {
        return inputs.AccelerationX;
    }

    public double getAccelerationY() {
        return inputs.AccelerationY;
    }

    public double getAccelerationZ() {
        return inputs.AccelerationZ;
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
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Gyro", inputs);
    }
}
