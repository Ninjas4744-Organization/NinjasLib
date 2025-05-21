package frc.lib.NinjasLib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.dataclasses.VisionConstants;
import frc.lib.NinjasLib.dataclasses.VisionOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVisionCamera extends VisionCamera<PhotonCamera> {
    private final PhotonCamera _camera;
    private final PhotonPoseEstimator _estimator;
    private List<PhotonTrackedTarget> _targets;
    private boolean disconnected = false;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public PhotonVisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);

        _camera = new PhotonCamera(name);

        _estimator = new PhotonPoseEstimator(
                _constants.fieldLayoutGetter.getFieldLayout(List.of()),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraPose);
        _estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        _output.cameraName = name;
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    @Override
    public VisionOutput Update() {
        if(disconnected)
            return _output;

        PhotonPipelineResult result;
        try {
            List<PhotonPipelineResult> results = _camera.getAllUnreadResults();
            if(results.isEmpty())
                return _output;
            result = results.get(results.size() - 1);
        } catch (Exception e) {
            System.out.println("Camera " + getName() + " disconnected");
            System.out.println(e.getMessage());

            disconnected = true;
            _output.hasTargets = false;
            _output.amountOfTargets = 0;

            return _output;
        }

        _estimator.setFieldTags(_constants.fieldLayoutGetter.getFieldLayout(_ignoredTags));
        Optional<EstimatedRobotPose> currentPose = _estimator.update(result);

        _output.hasTargets = result.hasTargets();
        _output.amountOfTargets = result.getTargets().size();
        if (currentPose.isEmpty()) return _output;

        _targets = currentPose.get().targetsUsed;
        findMinMax(_output);

        if (_output.maxAmbiguity < _constants.maxAmbiguity && _output.closestTagDist < _constants.maxDistance) {
            _output.timestamp = currentPose.get().timestampSeconds;
            _output.robotPose = currentPose.get().estimatedPose.toPose2d();
        } else {
            _output.hasTargets = false;
            _output.amountOfTargets = 0;
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
                output.closestTag = _tags.get(target.getFiducialId());
            }

            if (distance > output.farthestTagDist) {
                output.farthestTagDist = distance;
                output.farthestTag = _tags.get(target.getFiducialId());
            }

            if (ambiguity > output.maxAmbiguity) {
                output.maxAmbiguity = ambiguity;
                output.maxAmbiguityTag = _tags.get(target.getFiducialId());
            }
        }
    }

    /**
     * @return The camera processor that is being used by this VisionCamera
     */
    @Override
    public PhotonCamera getCamera() {
        return _camera;
    }

    /**
     * @return name of the camera
     */
    @Override
    public String getName() {
        return _camera.getName();
    }
}
