package com.ninjas4744.lib.DataClasses;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionEstimation {
	/** The pose of the robot */
	public Pose2d pose;

	/** The time this pose was detected by */
	public double timestamp;

	/** Whether or not this the camera that returned this estimation has targets */
	public boolean hasTargets;

	/**
	 * Creates a new VisionEstimation
	 *
	 * @param pose - The pose of the robot
	 * @param timestamp - The time this pose was detected by
	 * @param hasTargets - Whether or not this the camera that returned this estimation has targets
	 */
	public VisionEstimation(Pose2d pose, double timestamp, boolean hasTargets) {
		this.pose = pose;
		this.timestamp = timestamp;
		this.hasTargets = hasTargets;
	}
}
