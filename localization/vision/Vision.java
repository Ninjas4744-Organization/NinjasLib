package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.NinjasLogger;
import frc.robot.Robot;
import org.photonvision.simulation.VisionSystemSim;

import java.util.*;

public class Vision {
	private static Vision instance;
	private HashMap<String, VisionCameraIO> cameras;
	private HashMap<String, VisionOutput[]> outputs;
	private VisionSystemSim sim;
	private VisionConstants constants;

	public static Vision getInstance() {
		if (instance == null)
			throw new RuntimeException("Vision instance not set. Set vision instance by setInstance(Vision vision).");
		return instance;
	}

	public static void setInstance(Vision vision) {
		instance = vision;
	}

	public Vision(VisionConstants constants) {
		String[] camerasNames = constants.cameras.keySet().toArray(new String[0]);

		this.constants = constants;
		if (Robot.isSimulation()) {
			sim = new VisionSystemSim("main");
			sim.addAprilTags(constants.fieldLayoutGetter.getFieldLayout(List.of()).get());
		}

		cameras = new HashMap<>();
		outputs = new HashMap<>();
		for (int i = 0; i < constants.cameras.size(); i++) {
			Pair<Transform3d, VisionConstants.CameraType> cameraInfo = constants.cameras.get(camerasNames[i]);

			if (Robot.isReal()) {
				if (cameraInfo.getSecond() == VisionConstants.CameraType.PhotonVision)
					cameras.put(camerasNames[i], new PhotonVisionCameraIO(camerasNames[i], cameraInfo.getFirst(), constants));
				else
					cameras.put(camerasNames[i], new LimelightVisionCameraIO(camerasNames[i], cameraInfo.getFirst(), constants));
			} else {
				PhotonVisionSimCameraIO cam = new PhotonVisionSimCameraIO(camerasNames[i], cameraInfo.getFirst(), constants);
				cameras.put(camerasNames[i], cam);
				sim.addCamera(cam.getSim(), cameraInfo.getFirst());
			}

			outputs.put(camerasNames[i], new VisionOutput[0]);
		}
	}

	public void periodic() {
		for (String name : cameras.keySet()) {
			outputs.put(name, cameras.get(name).update());
			NinjasLogger.log("Vision/" + name, outputs.get(name));
		}

		if (Robot.isSimulation())
			sim.update(constants.robotPoseSupplier.get());
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
	 */
	public VisionOutput[] getVisionOutputs() {
		List<VisionOutput> outputsArr = new ArrayList<>();
		for (VisionOutput[] arr : outputs.values()) {
			Collections.addAll(outputsArr, arr);
		}
		return outputsArr.toArray(new VisionOutput[0]);
	}

	/**
	 * @param camera - the name of the name of the camera to get info from
	 * @return distance from the closest tag to this camera
	 */
	public double getClosestTargetDistance(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].closestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return closest tag to this camera
	 */
	public int getClosestTarget(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].closestTargetId;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return distance from the farthest tag to this camera
	 */
	public double getFarthestTargetDistance(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].farthestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return ambiguity of the most ambiguous tag from this camera
	 */
	public double getMaxAmbiguity(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].ambiguity;
	}

	public Transform3d getCameraToClosestTargetTransform(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].cameraToClosestTargetTransform;
	}

	public Transform3d[] getCameraToTargetsTransforms(String camera) {
		return outputs.get(camera)[outputs.get(camera).length - 1].cameraToTargetsTransforms;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return if this camera has targets
	 */
	public boolean hasTargets(String camera) {
		VisionOutput[] cameraOutputs = outputs.get(camera);
		if (cameraOutputs == null || cameraOutputs.length == 0) {
			return false;
		}

		return Arrays.stream(cameraOutputs).anyMatch(output -> output.hasTargets);
	}

	/**
	 * @return if any of the cameras have targets
	 */
	public boolean hasTargets() {
		for (String camera : cameras.keySet()) {
			if (hasTargets(camera)) return true;
		}
		return false;
	}

	/**
	 * If any camera sees this apriltag it will ignore it and not count it in the vision processing
	 *
	 * @param id ID of the apriltag to ignore
	 */
	public void ignoreTag(int id) {
		for (String name : cameras.keySet())
			cameras.get(name).ignoreTag(id);
	}

	public void unIgnoreTag(int id) {
		for (String name : cameras.keySet())
			cameras.get(name).unIgnoreTag(id);
	}
}
