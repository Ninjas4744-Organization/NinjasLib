package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        double Speed;
        Rotation2d Angle;
        Rotation2d AbsoluteAngle;
    }

    default SwerveModuleState getState() {
        return null;
    }

    default SwerveModulePosition getPosition() {
        return null;
    }

    default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    }

    default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }

    default int getModuleNumber() {
        return 0;
    }
}
