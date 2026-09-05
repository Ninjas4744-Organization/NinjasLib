package frc.lib.NinjasLib.swerve.constants;

import frc.lib.NinjasLib.controllers.constants.ControlConstants;

/**
 * Configuration for a higher-level swerve auto-driving controller (e.g. drive-to-pose), layering PID
 * gains for translation and rotation on top of the base {@link SwerveConstants}.
 */
public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** Swerve auto driving PID constants */
    public ControlConstants drivePIDConstants = ControlConstants.createPID(0, 0, 0, 0);

    /** Swerve auto driving angle PID constants */
    public ControlConstants rotationPIDConstants = ControlConstants.createPID(0, 0, 0, 0);

    /** @param swerveConstants the base swerve constants to control */
    public SwerveControllerConstants withSwerveConstants(SwerveConstants swerveConstants) {
        this.swerveConstants = swerveConstants;
        return this;
    }

    /** @param drivePIDConstants PID gains used to close the loop on translation during auto-driving */
    public SwerveControllerConstants withDrivePID(ControlConstants drivePIDConstants) {
        this.drivePIDConstants = drivePIDConstants;
        return this;
    }

    /** @param rotationPIDConstants PID gains used to close the loop on heading during auto-driving */
    public SwerveControllerConstants withRotationPID(ControlConstants rotationPIDConstants) {
        this.rotationPIDConstants = rotationPIDConstants;
        return this;
    }
}