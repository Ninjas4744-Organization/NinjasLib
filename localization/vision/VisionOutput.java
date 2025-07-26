package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionOutput {
	/** The pose of the robot */
	public Pose2d robotPose;

	/** The time at which the pose was detected */
	public double timestamp;

	/** The tag which was detected the closest */
	public AprilTag closestTarget;

	public AprilTag[] targets;

	public Transform3d cameraToClosestTargetTransform;

	public Transform3d[] cameraToTargetsTransforms;

	/** The ambiguity of the tag which was detected most ambiguously */
	public double maxAmbiguity;

	/** The distance from the camera of the tag which was detected the farthest */
	public double farthestTargetDist;

	/** The distance from the camera of the tag which was detected the closest */
	public double closestTargetDist;

	/** Whether the camera detected any tags */
	public boolean hasTargets;

	/** How many targets the camera detected */
	public int amountOfTargets;

	/** The name of the camera */
	public String cameraName;
}
