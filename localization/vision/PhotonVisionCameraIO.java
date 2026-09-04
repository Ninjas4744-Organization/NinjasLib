package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.util.NinjasLogger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;

/**
 * {@link VisionCameraIO} implementation for a real, physically-connected PhotonVision camera.
 * Wraps a {@code PhotonCamera}/{@code PhotonPoseEstimator} pair to turn unread pipeline results
 * into multi-tag {@link VisionOutput} pose estimates, and tolerates the camera disconnecting.
 * For a simulated PhotonVision camera, see the subclass {@link PhotonVisionSimCameraIO}.
 */
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
     * Creates a PhotonVision camera IO and builds its {@code PhotonPoseEstimator} from the current
     * field layout (via {@code constants.fieldLayoutGetter}).
     *
     * @param name       name of the camera, matching its name in the PhotonVision UI/coprocessor
     * @param cameraPose location of the camera on the robot (from center, positive x forward,
     *                   positive y left, and positive angle is counterclockwise)
     * @param constants  shared vision configuration, used to obtain the AprilTag field layout
     */
    public PhotonVisionCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        ignoredTags = new ArrayList<>();
        fillTagsMap();

        camera = new PhotonCamera(name);

        estimator = new PhotonPoseEstimator(constants.fieldLayoutGetter.getFieldLayout(List.of()).get(), cameraPose);
        estimator.setFieldTags(constants.fieldLayoutGetter.getFieldLayout(ignoredTags).get());
    }

    /**
     * Drains all unread pipeline results from the camera and turns each one into a
     * {@link VisionOutput} carrying a multi-tag pose estimate (via
     * {@code PhotonPoseEstimator.estimateCoprocMultiTagPose}) plus the per-target data used to
     * populate {@link Vision}'s target-distance/ambiguity accessors. This is the method
     * {@link Vision#periodic()} calls each loop to pull fresh data from this camera.
     * <p>
     * If reading from the camera throws (e.g. it is unplugged), the camera is marked disconnected
     * and every subsequent call returns an empty array until the object is recreated.
     *
     * @return one {@link VisionOutput} per unread pipeline result, or an empty array if there were
     * no new results or the camera is disconnected
     */
    public VisionOutput[] update() {
        if(disconnected)
            return new VisionOutput[0];

        List<VisionOutput> outputs;

        List<PhotonPipelineResult> results;
        try {
            results = camera.getAllUnreadResults();
            if(results.isEmpty())
                return new VisionOutput[0];

            outputs = new ArrayList<>();
            for (int i = 0; i < results.size(); i++) {
                outputs.add(new VisionOutput());
                outputs.get(i).cameraName = cameraName;
            }
        } catch (Exception e) {
            NinjasLogger.logEvent("Camera " + cameraName + " disconnected: " + e.getMessage());

            disconnected = true;
            return new VisionOutput[0];
        }

        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> currentPose = estimator.estimateCoprocMultiTagPose(results.get(i));

            outputs.get(i).hasTargets = results.get(i).hasTargets();
            outputs.get(i).amountOfTargets = results.get(i).getTargets().size();
            outputs.get(i).latency = results.get(i).metadata.getLatencyMillis() / 1000;

            if (currentPose.isEmpty())
                continue;

            targets = currentPose.get().targetsUsed;
            analyze(outputs.get(i));

            outputs.get(i).timestamp = currentPose.get().timestampSeconds;
            outputs.get(i).robotPose = currentPose.get().estimatedPose.toPose2d();
        }

        return outputs.toArray(new VisionOutput[0]);
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
     *
     * @param id the id of the apriltag to ignore
     */
    @Override
    public void ignoreTag(int id) {
        ignoredTags.add(id);
        fillTagsMap();
        estimator.setFieldTags(constants.fieldLayoutGetter.getFieldLayout(ignoredTags).get());
    }

    /**
     * Removes an apriltag from the ignored apriltags list, allowing it to be used again.
     *
     * @param id the id of the apriltag to stop ignoring
     */
    @Override
    public void unIgnoreTag(int id) {
        ignoredTags.remove((Integer) id);
        fillTagsMap();
        estimator.setFieldTags(constants.fieldLayoutGetter.getFieldLayout(ignoredTags).get());
    }

    private void fillTagsMap() {
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).get().getTags())
            tags.put(tag.ID, tag);
    }
}
