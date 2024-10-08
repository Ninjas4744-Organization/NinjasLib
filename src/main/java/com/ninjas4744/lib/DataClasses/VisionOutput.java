package com.ninjas4744.lib.DataClasses;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionOutput {
	/** The pose of the robot */
	public Pose2d robotPose;

	/** The time at which the pose was detected */
	public double timestamp;

	/** The tag which was detected most ambiguously */
	public AprilTag maxAmbiguityTag;

	/** The tag which was detected the farthest */
	public AprilTag farthestTag;

	/** The tag which was detected the closest */
	public AprilTag closestTag = new AprilTag(1, new Pose3d());

	/** The ambiguity of the tag which was detected most ambiguously */
	public double maxAmbiguity;

	/** The distance from the camera of the tag which was detected the farthest */
	public double farthestTagDist;

	/** The distance from the camera of the tag which was detected the closest */
	public double closestTagDist = -1;

	/** Whether the camera detected any tags */
	public boolean hasTargets = false;
}
