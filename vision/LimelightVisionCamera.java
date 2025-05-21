package frc.lib.NinjasLib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.dataclasses.VisionConstants;
import frc.lib.NinjasLib.dataclasses.VisionOutput;

public class LimelightVisionCamera extends VisionCamera<LimelightHelpers> {
    private final String cameraName;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public LimelightVisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);

        cameraName = name;

//        LimelightHelpers.setCameraPose_RobotSpace(cameraName,
//                cameraPose.getX(),    // Forward offset (meters)
//                cameraPose.getY(),    // Side offset (meters)
//                cameraPose.getZ(),    // Height offset (meters)
//                cameraPose.getRotation().getX(),    // Roll (degrees)
//                cameraPose.getRotation().getY(),   // Pitch (degrees)
//                cameraPose.getRotation().getZ()     // Yaw (degrees)
//        );
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    @Override
    public VisionOutput Update() {
        LimelightHelpers.PoseEstimate estimation = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

        if(estimation == null)
            return _output;

        _output.hasTargets = estimation.tagCount > 0;
        _output.amountOfTargets = estimation.tagCount;

        if (!_output.hasTargets)
            return _output;

        for (var target : estimation.rawFiducials)
            if(!_tags.containsKey(target.id))
                return _output;

        findMinMax(estimation);

        if (_output.maxAmbiguity < _constants.maxAmbiguity && _output.closestTagDist < _constants.maxDistance) {
            _output.robotPose = estimation.pose;
            _output.timestamp = estimation.timestampSeconds;
        } else {
            _output.hasTargets = false;
            _output.amountOfTargets = 0;
        }

        return _output;
    }

    private void findMinMax(LimelightHelpers.PoseEstimate estimation){
        _output.closestTagDist = Double.MAX_VALUE;
        _output.farthestTagDist = 0;
        _output.maxAmbiguity = 0;

        for (var target : estimation.rawFiducials) {
            double distance = target.distToCamera;

            if (distance < _output.closestTagDist) {
                _output.closestTagDist = distance;
                _output.closestTag = _tags.get(target.id);
            }

            if (distance > _output.farthestTagDist) {
                _output.farthestTagDist = distance;
                _output.farthestTag = _tags.get(target.id);
            }
        }
    }

    /**
     * @return The camera processor that is being used by this VisionCamera
     */
    @Override
    public LimelightHelpers getCamera() {
        return new LimelightHelpers();
    }

    /**
     * @return name of the camera
     */
    @Override
    public String getName() {
        return cameraName;
    }
}
