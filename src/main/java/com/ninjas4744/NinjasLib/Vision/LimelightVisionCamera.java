package com.ninjas4744.NinjasLib.Vision;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

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

        LimelightHelpers.setCameraPose_RobotSpace(cameraName,
                cameraPose.getX(),    // Forward offset (meters)
                cameraPose.getY(),    // Side offset (meters)
                cameraPose.getZ(),    // Height offset (meters)
                cameraPose.getRotation().getX(),    // Roll (degrees)
                cameraPose.getRotation().getY(),   // Pitch (degrees)
                cameraPose.getRotation().getZ()     // Yaw (degrees)
        );
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    @Override
    public VisionOutput Update() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(cameraName);

        _output.hasTargets = results.valid;
        _output.amountOfTargets = results.targets_Fiducials.length;

        if (!_output.hasTargets)
            return _output;

        findMinMax(results);

        if (_output.maxAmbiguity < _constants.maxAmbiguity && _output.closestTagDist < _constants.maxDistance) {
            _output.robotPose = RobotStateIO.getAlliance() == DriverStation.Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiBlue(cameraName) : LimelightHelpers.getBotPose2d_wpiRed(cameraName);
            _output.timestamp = results.timestamp_RIOFPGA_capture;
        } else {
            _output.hasTargets = false;
            _output.amountOfTargets = 0;
        }

        return _output;
    }

    private void findMinMax(LimelightHelpers.LimelightResults results){
        _output.closestTagDist = Double.MAX_VALUE;
        _output.farthestTagDist = 0;
        _output.maxAmbiguity = 0;

        for (var target : results.targets_Fiducials) {
            double distance = target.getCameraPose_TargetSpace2D().getTranslation().getNorm();

            if (distance < _output.closestTagDist) {
                _output.closestTagDist = distance;
                _output.closestTag = _tags.get((int)target.fiducialID);
            }

            if (distance > _output.farthestTagDist) {
                _output.farthestTagDist = distance;
                _output.farthestTag = _tags.get((int)target.fiducialID);
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
