package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    class SwerveModuleIOInputs {
        public int moduleNumber;
        public SwerveModuleState desiredState = new SwerveModuleState();
        public SwerveModuleState state = new SwerveModuleState();
        public SwerveModulePosition position = new SwerveModulePosition();
        public Rotation2d absolutePosition = Rotation2d.kZero;

        // Odometry Thread
        public Rotation2d[] angles = new Rotation2d[0];
        public double[] positions = new double[0];
        public double[] timestamps = new double[0];
    }

    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean preventJittering);
    SwerveModuleIOInputs update();
    void periodic();
}
