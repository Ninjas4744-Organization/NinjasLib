package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

import java.util.*;

public abstract class VisionCamera {
    protected List<VisionOutput> outputs;
    protected final List<Integer> ignoredTags;
    protected final VisionConstants constants;
    protected Map<Integer, AprilTag> tags;
    protected String cameraName;

    @AutoLog
    public static class VisionCameraInputs {
        int DataPoints;
        Pose2d RobotPose;
        double Timestamp;

        int ClosestTargetId;
        Pose2d ClosestTargetPose;
        int[] TargetsIds;
        Pose2d[] TargetsPoses;

        Transform3d CameraToClosestTargetTransform;
        Transform3d[] CameraToTargetsTransforms;

        double MaxAmbiguity;
        double FarthestTargetDist;
        double ClosestTargetDist;

        boolean HasTargets;
        int AmountOfTargets;

        int[] IgnoredTags;
    }

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public VisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        outputs = new ArrayList<>();
        ignoredTags = new ArrayList<>();
        fillTagsMap();
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    public abstract List<VisionOutput> update();

    public void updateInputs(VisionCameraInputsAutoLogged inputs){
        inputs.DataPoints = outputs.size();
        inputs.IgnoredTags = ignoredTags.stream().mapToInt(Integer::intValue).toArray();

        if(!outputs.isEmpty()){
            VisionOutput output = outputs.get(outputs.size() - 1);

            inputs.RobotPose = output.robotPose;
            inputs.Timestamp = output.timestamp;

            inputs.ClosestTargetId = output.closestTarget.ID;
            inputs.ClosestTargetDist = output.closestTargetDist;
            inputs.TargetsIds = Arrays.stream(output.targets).mapToInt(x -> x.ID).toArray();
            inputs.TargetsPoses = Arrays.stream(output.targets).map(x -> x.pose.toPose2d()).toArray(Pose2d[]::new);

            inputs.CameraToClosestTargetTransform = output.cameraToClosestTargetTransform;
            inputs.CameraToTargetsTransforms = output.cameraToTargetsTransforms;

            inputs.MaxAmbiguity = output.maxAmbiguity;
            inputs.FarthestTargetDist = output.farthestTargetDist;

            inputs.HasTargets = output.hasTargets;
            inputs.AmountOfTargets = output.amountOfTargets;
        }
    }

    /**
     * @return name of the camera
     */
    public abstract String getName();

    /**
     * Adds an apriltag to the ignored apriltags list. If the camera sees a tag in the ignored list, it ignores it.
     * @param id the id of the apriltag to ignore
     */
    public void ignoreTag(int id) {
        ignoredTags.add(id);
        fillTagsMap();
    }

    private void fillTagsMap(){
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).getTags())
            tags.put(tag.ID, tag);
    }
}
