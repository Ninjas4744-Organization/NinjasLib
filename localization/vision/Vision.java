package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Vision extends SubsystemBase {
	private static Vision instance;
	private HashMap<String, List<VisionOutput>> outputs;
	private VisionCamera[] cameras;
	private HashMap<String, Integer> cameraNameToIndex;
	private HashMap<String, VisionCameraInputsAutoLogged> inputs;

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

		cameras = new VisionCamera[camerasNames.length];
		cameraNameToIndex = new HashMap<>();
		for (int i = 0; i < constants.cameras.size(); i++) {
			cameraNameToIndex.put(camerasNames[i], i);
			if (constants.cameras.get(camerasNames[i]).getSecond() == VisionConstants.CameraType.PhotonVision)
				cameras[i] = new PhotonVisionCamera(camerasNames[i], constants.cameras.get(camerasNames[i]).getFirst(), constants);
			else
				cameras[i] = new LimelightVisionCamera(camerasNames[i], constants.cameras.get(camerasNames[i]).getFirst(), constants);
		}

		outputs = new HashMap<>();
		inputs = new HashMap<>();
		for (String name : camerasNames){
			outputs.put(name, new ArrayList<>());
			inputs.put(name, new VisionCameraInputsAutoLogged());
		}
	}

	@Override
	public void periodic() {
		for (VisionCamera camera : cameras) {
			outputs.put(camera.getName(), camera.update());

			camera.updateInputs(inputs.get(camera.getName()));
			Logger.processInputs("Vision/" + camera.getName(), inputs.get(camera.getName()));
		}
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
	 */
	public VisionOutput[] getVisionEstimations() {
		List<VisionOutput> estimations = new ArrayList<>();
		for (List<VisionOutput> cameraOutputs : outputs.values())
			estimations.addAll(cameraOutputs);

		return estimations.toArray(new VisionOutput[0]);
	}

	/**
	 * @param camera - the name of the name of the camera to get info from
	 * @return distance from the closest tag to this camera
	 */
	public double getClosestTargetDistance(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).closestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return closest tag to this camera
	 */
	public AprilTag getClosestTarget(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).closestTarget;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return distance from the farthest tag to this camera
	 */
	public double getFarthestTargetDistance(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).farthestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return ambiguity of the most ambiguous tag from this camera
	 */
	public double getMaxAmbiguity(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).maxAmbiguity;
	}

	public Transform3d getCameraToClosestTargetTransform(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).cameraToClosestTargetTransform;
	}

	public Transform3d[] getCameraToTargetsTransforms(String camera) {
		return outputs.get(camera).get(outputs.get(camera).size() - 1).cameraToTargetsTransforms;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return if this camera has targets
	 */
	public boolean hasTargets(String camera) {
		if (outputs.get(camera).isEmpty())
			return false;
		return outputs.get(camera).get(outputs.get(camera).size() - 1).hasTargets;
	}

	/**
	 * @return if any of the cameras have targets
	 */
	public boolean hasTargets() {
		for (String camera : outputs.keySet()) {
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
		for (VisionCamera camera : cameras) camera.ignoreTag(id);
	}

	/**
	 * @param name The name of the camera
	 * @return Vision camera processor
	 */
	public VisionCamera getCamera(String name) {
		return cameras[cameraNameToIndex.get(name)];
	}
}
