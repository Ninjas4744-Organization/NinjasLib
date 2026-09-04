package frc.lib.NinjasLib.swerve.constants;

import frc.lib.NinjasLib.controllers.constants.ControlConstants;

public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** Swerve auto driving PID constants */
    public ControlConstants drivePIDConstants;

    /** Swerve auto driving angle PID constants */
    public ControlConstants rotationPIDConstants;

    public SwerveControllerConstants withSwerveConstants(SwerveConstants swerveConstants) {
        this.swerveConstants = swerveConstants;
        return this;
    }

    public SwerveControllerConstants withDrivePID(ControlConstants drivePIDConstants) {
        this.drivePIDConstants = drivePIDConstants;
        return this;
    }

    public SwerveControllerConstants withRotationPID(ControlConstants rotationPIDConstants) {
        this.rotationPIDConstants = rotationPIDConstants;
        return this;
    }
}