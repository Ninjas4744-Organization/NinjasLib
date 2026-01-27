package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public int ModuleNumber;
        public SwerveModuleState DesiredState = new SwerveModuleState();
        public SwerveModuleState State = new SwerveModuleState();
        public SwerveModulePosition Position = new SwerveModulePosition();
        public Rotation2d AbsolutePosition = Rotation2d.kZero;

        // Odometry Thread
        public Rotation2d[] Angles = new Rotation2d[0];
        public double[] Positions = new double[0];
        public double[] Timestamps = new double[0];
    }

    default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    }

    default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
