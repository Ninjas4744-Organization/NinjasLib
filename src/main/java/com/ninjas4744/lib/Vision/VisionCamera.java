package com.ninjas4744.lib.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ninjas4744.lib.DataClasses.VisionOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class VisionCamera {
	private final PhotonCamera _camera;
	private final PhotonPoseEstimator _estimator;
	private List<PhotonTrackedTarget> _targets;
	private final VisionOutput _output;
	private final List<Integer> _ignoredTags;

	private final Function<Optional<List<Integer>>, AprilTagFieldLayout> _getFieldLayout;
	private final double _maxAmbiguity;

	/**
	 * @param name Name of the camera.
	 * @param cameraPose Location of the camera on the robot (from center, positive x forward,
	 *     positive y left, and positive angle is counterclockwise).
	 */
	public VisionCamera(String name, Transform3d cameraPose, Function<Optional<List<Integer>>, AprilTagFieldLayout> getFieldLayout, double maxAmbiguity) {
		_getFieldLayout = getFieldLayout;
		_maxAmbiguity = maxAmbiguity;

		_camera = new PhotonCamera(name);

		_estimator = new PhotonPoseEstimator(
				_getFieldLayout.apply(Optional.empty()),
				PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				_camera,
				cameraPose);
		_estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

		_output = new VisionOutput();

		_ignoredTags = new ArrayList<>();
	}

	/**
	 * Updates the results of this camera, should run on periodic
	 * @return The vision output of this camera
	 */
	public VisionOutput Update() {
		PhotonPipelineResult result = _camera.getLatestResult();
		_estimator.setFieldTags(_getFieldLayout.apply(Optional.of(_ignoredTags)));
		Optional<EstimatedRobotPose> currentPose = _estimator.update(result);

		_output.hasTargets = result.hasTargets();
		SmartDashboard.putBoolean("Vision Has Targets", _output.hasTargets);
		if (currentPose.isEmpty()) return _output;

		_targets = currentPose.get().targetsUsed;
		findMinMax(_output);

		if (_output.maxAmbiguity < _maxAmbiguity) {
			_output.timestamp = currentPose.get().timestampSeconds;

			_output.robotPose = new Pose2d(
					currentPose.get().estimatedPose.getX(),
					currentPose.get().estimatedPose.getY(),
					Rotation2d.fromRadians(
							currentPose.get().estimatedPose.getRotation().getZ()));
		}

		return _output;
	}

	private void findMinMax(VisionOutput output) {
		output.closestTagDist = Double.MAX_VALUE;
		output.farthestTagDist = 0;
		output.maxAmbiguity = 0;

		for (PhotonTrackedTarget target : _targets) {
			double distance = target.getBestCameraToTarget().getTranslation().getNorm();
			double ambiguity = target.getPoseAmbiguity();

			if (distance < output.closestTagDist) {
				output.closestTagDist = distance;
				output.closestTag = _getFieldLayout.apply(Optional.empty()).getTags().get(target.getFiducialId() - 1);
			}

			if (distance > output.farthestTagDist) {
				output.farthestTagDist = distance;
				output.farthestTag = _getFieldLayout.apply(Optional.empty()).getTags().get(target.getFiducialId() - 1);
			}

			if (ambiguity > output.maxAmbiguity) {
				output.maxAmbiguity = ambiguity;
				output.maxAmbiguityTag = _getFieldLayout.apply(Optional.empty()).getTags().get(target.getFiducialId() - 1);
			}
		}
	}

	/**
	 * @return The PhotonCamera that is being used by this VisionCamera
	 */
	public PhotonCamera getCamera() {
		return _camera;
	}

	/**
	 * @return name of the camera
	 */
	public String getName() {
		return _camera.getName();
	}

	public void ignoreTag(int id) {
		_ignoredTags.add(id);
	}
}
