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

	public ChassisSpeeds driverInput = new ChassisSpeeds(0, 0, 0);
	public ChassisSpeeds velocity = new ChassisSpeeds(0, 0, 0);
	public boolean fieldRelative = true;
	public Pose2d targetPose = new Pose2d();
	public Rotation2d sheer = new Rotation2d();
	public Rotation2d angle = new Rotation2d();
	public double phase = 0;
	public boolean isXDriverInput = false;
}
