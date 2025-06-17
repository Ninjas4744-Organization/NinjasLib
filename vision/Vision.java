package frc.lib.NinjasLib.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.dataclasses.VisionConstants;
import frc.lib.NinjasLib.dataclasses.VisionOutput;

import java.util.HashMap;

public class Vision extends SubsystemBase {
	private static Vision _instance;
	protected HashMap<String, VisionOutput> _outputs;
	protected VisionCamera[] _cameras;
	protected HashMap<String, Integer> _cameraNameToIndex;

	public static Vision getInstance() {
		if (_instance == null)
			throw new RuntimeException("VisionIO constants not given. Initialize VisionIO by setConstants(VisionConstants) first.");
		return _instance;
	}

	public static void setConstants(VisionConstants constants) {
		_instance = new Vision(constants);
	}

	protected Vision(VisionConstants constants) {
		String[] camerasNames = constants.cameras.keySet().toArray(new String[0]);

		_cameras = new VisionCamera[camerasNames.length];
		_cameraNameToIndex = new HashMap<>();
		for (int i = 0; i < constants.cameras.size(); i++) {
			_cameraNameToIndex.put(camerasNames[i], i);
			if (constants.cameras.get(camerasNames[i]).getSecond() == VisionConstants.CameraType.PhotonVision)
				_cameras[i] = new PhotonVisionCamera(camerasNames[i], constants.cameras.get(camerasNames[i]).getFirst(), constants);
			else
				_cameras[i] = new LimelightVisionCamera(camerasNames[i], constants.cameras.get(camerasNames[i]).getFirst(), constants);
		}

		_outputs = new HashMap<>();
		for (String name : camerasNames)
			_outputs.put(name, new VisionOutput());
	}

	@Override
	public void periodic() {
		for (VisionCamera camera : _cameras)
			_outputs.put(camera.getName(), camera.Update());
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
	 */
	public VisionOutput[] getVisionEstimations() {
		return _outputs.values().toArray(new VisionOutput[0]);
	}

	/**
	 * @param camera - the name of the name of the camera to get info from
	 * @return distance from the closest tag to this camera
	 */
	public double getClosestTagDistance(String camera) {
		return _outputs.get(camera).closestTagDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return closest tag to this camera
	 */
	public AprilTag getClosestTag(String camera) {
		return _outputs.get(camera).closestTag;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return distance from the farthest tag to this camera
	 */
	public double getFarthestTagDistance(String camera) {
		return _outputs.get(camera).farthestTagDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return farthest tag to this camera
	 */
	public AprilTag getFarthestTag(String camera) {
		return _outputs.get(camera).farthestTag;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return ambiguity of the most ambiguous tag from this camera
	 */
	public double getMostAmbiguousTagAmbiguity(String camera) {
		return _outputs.get(camera).maxAmbiguity;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return most ambiguous tag from this camera
	 */
	public AprilTag getMostAmbiguousTag(String camera) {
		return _outputs.get(camera).maxAmbiguityTag;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return if this camera has targets
	 */
	public boolean hasTargets(String camera) {
		return _outputs.get(camera).hasTargets;
	}

	/**
	 * @return if any of the cameras have targets
	 */
	public boolean hasTargets() {
		for (String camera : _outputs.keySet()) {
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
		for (VisionCamera camera : _cameras) camera.ignoreTag(id);
	}

	/**
	 * @param name The name of the camera
	 * @return Vision camera processor
	 */
	public VisionCamera getCamera(String name) {
		return _cameras[_cameraNameToIndex.get(name)];
	}
}
