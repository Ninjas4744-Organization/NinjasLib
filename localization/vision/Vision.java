package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.util.NinjasLogger;
import frc.robot.Robot;
import org.photonvision.simulation.VisionSystemSim;

import java.util.*;

/**
 * Top-level manager that aggregates every {@link VisionCameraIO} on the robot (any mix of
 * {@link LimelightVisionCameraIO}, {@link PhotonVisionCameraIO} and, in simulation,
 * {@link PhotonVisionSimCameraIO}) behind one API. Construct it once from a {@link VisionConstants}
 * describing the cameras, register it with {@link #setInstance(Vision)}, call {@link #periodic()}
 * every robot loop to pull fresh pose estimates, and read results back through the {@code getX}
 * accessors or {@link #getVisionOutputs()}.
 * <p>
 * If the supplied {@link VisionConstants} is incomplete, this class disables itself rather than
 * throwing: every accessor then returns a harmless default and {@link #periodic()} becomes a no-op.
 */
public class Vision {
	private static Vision instance;
	private HashMap<String, VisionCameraIO> cameras;
	private HashMap<String, VisionOutput[]> outputs;
	private VisionSystemSim sim;
	private VisionConstants constants;
	private boolean disabled = false;

	/**
	 * Gets the registered {@link Vision} singleton.
	 *
	 * @return the instance set via {@link #setInstance(Vision)}, or a disabled placeholder instance
	 * (logging a warning) if none has been set yet
	 */
	public static Vision get() {
		if (instance == null) {
			NinjasLogger.logEventImportant("Vision instance not set. Set vision instance by setInstance(Vision vision).");
			return new Vision(); // Disabled vision
		}
		return instance;
	}

	/**
	 * Registers the {@link Vision} singleton returned by subsequent calls to {@link #get()}.
	 * Should be called once during robot initialization.
	 *
	 * @param vision the instance to register
	 */
	public static void setInstance(Vision vision) {
		instance = vision;
	}

	private Vision() {
		disabled = true;
	}

	/**
	 * Builds every camera described in {@code constants} and, in simulation, a
	 * {@code VisionSystemSim} loaded with the field's AprilTags to drive them. If {@code constants}
	 * is missing its cameras, {@link VisionConstants#fieldLayoutGetter} or
	 * {@link VisionConstants#robotPoseSupplier}, vision is disabled instead of partially
	 * constructed.
	 *
	 * @param constants the cameras and shared configuration to build vision from
	 */
	public Vision(VisionConstants constants) {
		if (constants.cameras.isEmpty() || constants.fieldLayoutGetter == null || constants.robotPoseSupplier == null) {
			NinjasLogger.logEventImportant("Vision constructor parameters not set.");
			disabled = true;
			return;
		}

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

	/**
	 * Updates every camera and logs their outputs. Call this once per robot loop (e.g. from
	 * {@code Robot.robotPeriodic()}) - it is the sole point at which cameras are polled for new
	 * pose estimates, so every {@code getX} accessor below only reflects data as fresh as the last
	 * call to this method. In simulation, this also advances the shared {@code VisionSystemSim}
	 * using {@link VisionConstants#robotPoseSupplier} so simulated cameras see tags from the
	 * robot's actual simulated pose. No-op if vision is disabled.
	 */
	public void periodic() {
		if (disabled)
			return;

		for (String name : cameras.keySet()) {
			outputs.put(name, cameras.get(name).update());
			NinjasLogger.log("Vision/" + name + "/Outputs", outputs.get(name));
		}

		if (Robot.isSimulation())
			sim.update(constants.robotPoseSupplier.get());
	}

	/**
	 * Gets the full set of pose estimates produced by all cameras on the last {@link #periodic()}
	 * call. A camera may contribute zero, one, or (for PhotonVision, which can have multiple
	 * unread pipeline results per loop) several entries.
	 *
	 * @return every {@link VisionOutput} from every camera since the last {@link #periodic()} call
	 */
	public VisionOutput[] getVisionOutputs() {
		if (disabled)
			return new VisionOutput[0];

		List<VisionOutput> outputsArr = new ArrayList<>();
		for (VisionOutput[] arr : outputs.values()) {
			Collections.addAll(outputsArr, arr);
		}
		return outputsArr.toArray(new VisionOutput[0]);
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return distance from the closest tag to this camera on its last output, or {@code 1} if
	 * vision is disabled
	 */
	public double getClosestTargetDistance(String camera) {
		if (disabled)
			return 1;

		return outputs.get(camera)[outputs.get(camera).length - 1].closestTargetDist;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return id of the closest tag to this camera on its last output, or {@code 10} if vision is
	 * disabled
	 */
	public int getClosestTarget(String camera) {
		if (disabled)
			return 10;

		return outputs.get(camera)[outputs.get(camera).length - 1].closestTargetId;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return distance from the farthest tag to this camera on its last output, or {@code 1} if
	 * vision is disabled
	 */
	public double getFarthestTargetDistance(String camera) {
		if (disabled)
			return 1;

		return outputs.get(camera)[outputs.get(camera).length - 1].farthestTargetDist;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return {@link VisionOutput#ambiguity} from this camera's last output, or {@code 0} if
	 * vision is disabled
	 */
	public double getMaxAmbiguity(String camera) {
		if (disabled)
			return 0;

		return outputs.get(camera)[outputs.get(camera).length - 1].ambiguity;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return the camera-to-closest-target transform from this camera's last output, or the
	 * identity transform if vision is disabled
	 */
	public Transform3d getCameraToClosestTargetTransform(String camera) {
		if (disabled)
			return new Transform3d();

		return outputs.get(camera)[outputs.get(camera).length - 1].cameraToClosestTargetTransform;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return the camera-to-target transforms for every tag this camera detected in its last
	 * output, or an empty array if vision is disabled
	 */
	public Transform3d[] getCameraToTargetsTransforms(String camera) {
		if (disabled)
			return new Transform3d[0];

		return outputs.get(camera)[outputs.get(camera).length - 1].cameraToTargetsTransforms;
	}

	/**
	 * @param camera the name of the camera to get info from
	 * @return whether this camera reported any targets in its outputs from the last
	 * {@link #periodic()} call
	 */
	public boolean hasTargets(String camera) {
		if (disabled)
			return false;

		VisionOutput[] cameraOutputs = outputs.get(camera);
		if (cameraOutputs == null || cameraOutputs.length == 0) {
			return false;
		}

		return Arrays.stream(cameraOutputs).anyMatch(output -> output.hasTargets);
	}

	/**
	 * @return whether any camera reported targets in its outputs from the last
	 * {@link #periodic()} call
	 */
	public boolean hasTargets() {
		if (disabled)
			return false;

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
		if (disabled)
			return;

		for (String name : cameras.keySet())
			cameras.get(name).ignoreTag(id);
	}

	/**
	 * Reverses {@link #ignoreTag(int)} on every camera, allowing a previously ignored tag to be
	 * used again.
	 *
	 * @param id ID of the apriltag to stop ignoring
	 */
	public void unIgnoreTag(int id) {
		if (disabled)
			return;

		for (String name : cameras.keySet())
			cameras.get(name).unIgnoreTag(id);
	}
}
