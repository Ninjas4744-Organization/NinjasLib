package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;

public class PhotonVisionCameraIO implements VisionCameraIO {
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private List<PhotonTrackedTarget> targets;
    private boolean disconnected = false;
    private final List<Integer> ignoredTags;
    private final VisionConstants constants;
    private Map<Integer, AprilTag> tags;
    private String cameraName;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public PhotonVisionCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        ignoredTags = new ArrayList<>();
        fillTagsMap();

        camera = new PhotonCamera(name);

        estimator = new PhotonPoseEstimator(
            this.constants.fieldLayoutGetter.getFieldLayout(List.of()),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraPose);
        estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    /**
     * Updates the results of this camera, should run on periodic
     */
    @Override
    public void updateInputs(VisionCameraIOInputsAutoLogged inputs) {
        inputs.outputs = new VisionOutput[0];
        if(disconnected)
            return;

        List<VisionOutput> outputs;

        List<PhotonPipelineResult> results;
        try {
            results = camera.getAllUnreadResults();
            if(results.isEmpty())
                return;

            outputs = new ArrayList<>();
            for (int i = 0; i < results.size(); i++) {
                outputs.add(new VisionOutput());
                outputs.get(i).cameraName = cameraName;
            }
        } catch (Exception e) {
            System.out.println("Camera " + cameraName + " disconnected");
            System.out.println(e.getMessage());

            disconnected = true;
            return;
        }

        estimator.setFieldTags(constants.fieldLayoutGetter.getFieldLayout(ignoredTags));
        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> currentPose = estimator.update(results.get(i));

            outputs.get(i).hasTargets = results.get(i).hasTargets();
            outputs.get(i).amountOfTargets = results.get(i).getTargets().size();
            outputs.get(i).latency = results.get(i).metadata.getLatencyMillis() / 1000;

            if (currentPose.isEmpty())
                continue;

            targets = currentPose.get().targetsUsed;
            analyze(outputs.get(i));

            outputs.get(i).timestamp = currentPose.get().timestampSeconds;
            outputs.get(i).robotPose = currentPose.get().estimatedPose.toPose2d();
//            if (outputs.get(i).maxAmbiguity < constants.maxAmbiguity && outputs.get(i).closestTargetDist < constants.maxDistance) {
//                outputs.get(i).timestamp = currentPose.get().timestampSeconds;
//                outputs.get(i).robotPose = currentPose.get().estimatedPose.toPose2d();
//            } else {
//                outputs.get(i).hasTargets = false;
//                outputs.get(i).amountOfTargets = 0;
//            }
        }

        inputs.outputs = outputs.toArray(new VisionOutput[0]);
    }

    private void analyze(VisionOutput output) {
        output.closestTargetDist = Double.MAX_VALUE;
        output.farthestTargetDist = 0;
        output.ambiguity = 0;

        output.targetsIds = new int[targets.size()];
        output.targetsPoses = new Pose3d[targets.size()];
        output.cameraToTargetsTransforms = new Transform3d[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            output.targetsIds[i] = targets.get(i).getFiducialId();
            output.targetsPoses[i] = tags.get(targets.get(i).getFiducialId()).pose;
            output.cameraToTargetsTransforms[i] = targets.get(i).getBestCameraToTarget();

            double distance = targets.get(i).getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = targets.get(i).getPoseAmbiguity();

            if (distance < output.closestTargetDist) {
                output.closestTargetDist = distance;
                output.closestTargetId = targets.get(i).getFiducialId();
                output.closestTargetPose = tags.get(targets.get(i).getFiducialId()).pose;
                output.cameraToClosestTargetTransform = targets.get(i).getBestCameraToTarget();
            }

            if (distance > output.farthestTargetDist)
                output.farthestTargetDist = distance;

            if (ambiguity < output.ambiguity)
                output.ambiguity = ambiguity;
        }
    }

    /**
     * Adds an apriltag to the ignored apriltags list. If the camera sees a tag in the ignored list, it ignores it.
     * @param id the id of the apriltag to ignore
     */
    @Override
    public void ignoreTag(int id) {
        ignoredTags.add(id);
        fillTagsMap();
    }

    private void fillTagsMap() {
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).getTags())
            tags.put(tag.ID, tag);
    }
}
