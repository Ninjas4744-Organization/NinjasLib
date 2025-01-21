package com.ninjas4744.NinjasLib.DataClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDemand {
	public enum SwerveState {
		DEFAULT,
		VELOCITY,
		LOCKED_AXIS,
		LOOK_AT_TARGET,
		PATHFINDING,
		DRIVE_ASSIST
	}

	/** Used for DEFAULT, LOCKED_AXIS, LOOK_AT_TARGET, PATHFINDING, DRIVE_ASSIST */
	public ChassisSpeeds driverInput = new ChassisSpeeds(0, 0, 0);

	/** Used for VELOCITY */
	public ChassisSpeeds velocity = new ChassisSpeeds(0, 0, 0);

	/** Used for VELOCITY */
	public boolean fieldRelative = true;

	/** Used for LOOK_AT_TARGET, PATHFINDING, DRIVE_ASSIST */
	public Pose2d targetPose = new Pose2d();

	/** Used for LOOK_AT_TARGET */
	public Rotation2d angleOffset = new Rotation2d();

	/** Used for LOCKED_AXIS */
	public Rotation2d angle = new Rotation2d();

	/** Used for LOCKED_AXIS */
	public Pose2d point = new Pose2d();

	/** Whether to control locked axis with X of controller or Y of controller, Used for LOCKED_AXIS */
	public boolean isXDriverInput = false;

	/** Used for LOCKED_AXIS */
    public boolean invertDriverInput = false;
}
