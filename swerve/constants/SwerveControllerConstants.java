package frc.lib.NinjasLib.swerve.constants;

import edu.wpi.first.math.Pair;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;

public class SwerveControllerConstants {
    /** Regular swerve constants */
    public SwerveConstants swerveConstants;

    /** Swerve auto driving PID constants */
    public ControlConstants drivePIDConstants;

    /** Swerve auto driving angle PID constants */
    public ControlConstants rotationPIDConstants;

    /** The swerve rotation PID needs to know that 360deg and 0deg are the same so the rotation will work properly.
     * Choose 2 values that are equal in radians like 0, 360, -180, 180 or 270, -90. it's in radians so convert the deg to rad.
     * For some reason only one of the pairs will work for each robot so good luck!
     */
    public Pair<Double, Double> rotationPIDContinuousConnections;

    /**
     * Whether to make the swerve try to stay at its wanted angle with PID.
     * It will fight the random rotations caused by fast driving.
     */
    public boolean enableRotationPIDCorrection = false;
}
