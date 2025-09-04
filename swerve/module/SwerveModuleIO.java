package frc.lib.NinjasLib.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public int ModuleNumber;
        public SwerveModuleState State = new SwerveModuleState();
        public SwerveModulePosition Position = new SwerveModulePosition();
        public Rotation2d AbsoluteAngle = Rotation2d.kZero;

        // Odometry Thread
        public Rotation2d[] Angles;
        public double[] Positions;
        public double[] Timestamps;
    }

    default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    }

    default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
