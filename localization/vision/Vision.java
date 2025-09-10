package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Vision extends SubsystemBase {
	private static Vision instance;
	private final HashMap<String, VisionCameraIO> cameras;
    private final HashMap<String, VisionCameraIOInputsAutoLogged> inputs;
	private VisionSystemSim sim;
	private final VisionConstants constants;

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
		if (Robot.isSimulation() && !constants.isReplay) {
			sim = new VisionSystemSim("main");
			sim.addAprilTags(constants.fieldLayoutGetter.getFieldLayout(List.of()));
		}

		cameras = new HashMap<>();
		inputs = new HashMap<>();
		for (int i = 0; i < constants.cameras.size(); i++) {
			Pair<Transform3d, VisionConstants.CameraType> cameraInfo = constants.cameras.get(camerasNames[i]);

            try {
                if (Robot.isReal()) {
                    if (cameraInfo.getSecond() == VisionConstants.CameraType.PhotonVision)
                        cameras.put(camerasNames[i], new PhotonVisionCameraIO(camerasNames[i], cameraInfo.getFirst(), constants));
                    else
                        cameras.put(camerasNames[i], new LimelightVisionCameraIO(camerasNames[i], cameraInfo.getFirst(), constants));
                } else if (!constants.isReplay) {
                    PhotonVisionSimCameraIO cam = new PhotonVisionSimCameraIO(camerasNames[i], cameraInfo.getFirst(), constants);
                    cameras.put(camerasNames[i], cam);
                    sim.addCamera(cam.getSim(), cameraInfo.getFirst());
                } else
                    cameras.put(camerasNames[i], new VisionCameraIO() {
                    });
            } catch (Exception e) {
				System.out.println("Camera name: "+ camerasNames[i]+" not found");
				continue;
			}

            inputs.put(camerasNames[i], new VisionCameraIOInputsAutoLogged());
		}
	}

	@Override
	public void periodic() {
		for (String name : cameras.keySet()) {
			if (name == null) {
				continue;
			}

			cameras.get(name).updateInputs(inputs.get(name));
			Logger.processInputs("Vision/" + name, inputs.get(name));
		}

		if (Robot.isSimulation() && !constants.isReplay)
			sim.update(constants.robotPoseSupplier.get());
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
	 */
	public VisionOutput[] getVisionEstimations() {
		List<VisionOutput> estimations = new ArrayList<>();
		for (VisionCameraIOInputsAutoLogged i : inputs.values()) {
			for (VisionOutput o : i.outputs) {
				if (o.hasTargets)
					estimations.add(o);
			}
		}

		return estimations.toArray(new VisionOutput[0]);
	}

	/**
	 * @param camera - the name of the name of the camera to get info from
	 * @return distance from the closest tag to this camera
	 */
	public double getClosestTargetDistance(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return -1;
		}

		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].closestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return closest tag to this camera
	 */
	public int getClosestTarget(String camera) {
        if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return -1;
        }

		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].closestTargetId;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return distance from the farthest tag to this camera
	 */
	public double getFarthestTargetDistance(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return -1;
		}


		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].farthestTargetDist;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return ambiguity of the most ambiguous tag from this camera
	 */
	public double getMaxAmbiguity(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return -1;
		}


		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].maxAmbiguity;
	}

	public Transform3d getCameraToClosestTargetTransform(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return new Transform3d();
		}


		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].cameraToClosestTargetTransform;
	}

	public Transform3d[] getCameraToTargetsTransforms(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return new Transform3d[0];
		}


		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].cameraToTargetsTransforms;
	}

	/**
	 * @param camera - the name of the camera to get info from
	 * @return if this camera has targets
	 */
	public boolean hasTargets(String camera) {
		if (!cameras.containsKey(camera)) {
			System.out.println("Camera name: "+camera+" not found");
			return false;
		}

		if (inputs.get(camera).outputs.length == 0)
			return false;
		return inputs.get(camera).outputs[inputs.get(camera).outputs.length - 1].hasTargets;
	}

	/**
	 * @return if any of the cameras have targets
	 */
	public boolean hasTargets() {
		for (String camera : cameras.keySet()) {
			if (!cameras.containsKey(camera)) {
				System.out.println("Camera name: "+camera+" not found");
				continue;
			}

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
		for (String name : cameras.keySet()) {
			if (!cameras.containsKey(name)) {
				System.out.println("Camera name: "+name+" not found");
				continue;
			}

			cameras.get(name).ignoreTag(id);
		}
	}
}
