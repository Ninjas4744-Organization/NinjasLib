package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PhotonVisionCamera extends VisionCamera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private List<PhotonTrackedTarget> targets;
    private boolean disconnected = false;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public PhotonVisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);

        camera = new PhotonCamera(name);

        estimator = new PhotonPoseEstimator(
            this.constants.fieldLayoutGetter.getFieldLayout(List.of()),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraPose);
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    @Override
    public List<VisionOutput> update() {
        if(disconnected)
            return new ArrayList<>();

        List<PhotonPipelineResult> results;
        try {
            results = camera.getAllUnreadResults();
            if(results.isEmpty())
                return outputs;

            outputs = new ArrayList<>();
            for (int i = 0; i < results.size(); i++) {
                outputs.add(new VisionOutput());
                outputs.get(i).cameraName = cameraName;
            }
        } catch (Exception e) {
            System.out.println("Camera " + getName() + " disconnected");
            System.out.println(e.getMessage());

            disconnected = true;
            return new ArrayList<>();
        }

        estimator.setFieldTags(constants.fieldLayoutGetter.getFieldLayout(ignoredTags));
        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> currentPose = estimator.update(results.get(i));

            outputs.get(i).hasTargets = results.get(i).hasTargets();
            outputs.get(i).amountOfTargets = results.get(i).getTargets().size();

            if (currentPose.isEmpty())
                return new ArrayList<>();

            targets = currentPose.get().targetsUsed;
            analyze(outputs.get(i));

            if (outputs.get(i).maxAmbiguity < constants.maxAmbiguity && outputs.get(i).closestTargetDist < constants.maxDistance) {
                outputs.get(i).timestamp = currentPose.get().timestampSeconds;
                outputs.get(i).robotPose = currentPose.get().estimatedPose.toPose2d();
            } else {
                outputs.get(i).hasTargets = false;
                outputs.get(i).amountOfTargets = 0;
            }
        }

        return outputs;
    }

    private void analyze(VisionOutput output) {
        output.closestTargetDist = Double.MAX_VALUE;
        output.farthestTargetDist = 0;
        output.maxAmbiguity = 0;

        output.targets = new AprilTag[targets.size()];
        output.cameraToTargetsTransforms = new Transform3d[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            output.targets[i] = tags.get(targets.get(i).getFiducialId());
            output.cameraToTargetsTransforms[i] = targets.get(i).getBestCameraToTarget();

            double distance = targets.get(i).getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = targets.get(i).getPoseAmbiguity();

            if (distance < output.closestTargetDist) {
                output.closestTargetDist = distance;
                output.closestTarget = tags.get(targets.get(i).getFiducialId());
                output.cameraToClosestTargetTransform = targets.get(i).getBestCameraToTarget();
            }

            if (distance > output.farthestTargetDist)
                output.farthestTargetDist = distance;

            if (ambiguity > output.maxAmbiguity)
                output.maxAmbiguity = ambiguity;
        }
    }

    /**
     * @return The camera processor that is being used by this VisionCamera
     */
    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * @return name of the camera
     */
    @Override
    public String getName() {
        return camera.getName();
    }
}
