package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.localization.RobotPose;
import frc.lib.NinjasLib.swerve.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * {@link VisionCameraIO} implementation for a real Limelight smart camera. Reads MegaTag2 (and
 * MegaTag1, for comparison) pose estimates over NetworkTables via {@code LimelightHelpers}, and
 * feeds the current gyro yaw to the Limelight so it can compute MegaTag2 estimates on-device.
 */
public class LimelightVisionCameraIO implements VisionCameraIO {
    private final String cameraName;
    private LimelightHelpers.RawFiducial[] targets;
    private final List<Integer> ignoredTags;
    private final VisionConstants constants;
    private Map<Integer, AprilTag> tags;

    /**
     * Creates a Limelight camera IO and disables the Limelight's internal IMU fusion mode.
     *
     * @param name       the Limelight's configured network table name
     * @param cameraPose unused - a Limelight computes its own pose estimate on-device, so its
     *                   mounting transform is configured on the device itself, not here
     * @param constants  shared vision configuration, used to obtain the AprilTag field layout
     */
    public LimelightVisionCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        LimelightHelpers.SetIMUMode(cameraName, 0);
        ignoredTags = new ArrayList<>();
        fillTagsMap();
    }

    /**
     * Reads the latest MegaTag2 (and MegaTag1) pose estimates from the Limelight and packages them
     * into a single {@link VisionOutput}. Before reading, this pushes the robot's current gyro yaw
     * (adjusted for alliance, since MegaTag2 needs field-relative heading) to the Limelight, since
     * MegaTag2 requires external orientation input to resolve pose. This is the method
     * {@link Vision#periodic()} calls each loop to pull fresh data from this camera.
     *
     * @return an array containing a single {@link VisionOutput}, or an empty array if the alliance
     * is not yet known or no MegaTag2 estimate is available
     */
    public VisionOutput[] update() {
        List<VisionOutput> outputs = new ArrayList<>();

        if (RobotPose.getAlliance().isEmpty())
            return new VisionOutput[0];

        Rotation2d robotYaw = Swerve.get().getGyro().getYaw();
        if (RobotPose.getAlliance().get() == DriverStation.Alliance.Red) {
            robotYaw = robotYaw.rotateBy(Rotation2d.k180deg);
        }
        LimelightHelpers.SetRobotOrientation(cameraName, robotYaw.getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate = (RobotPose.getAlliance().get() == DriverStation.Alliance.Blue)
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraName);

        LimelightHelpers.PoseEstimate estimateMegaTag1 = (RobotPose.getAlliance().get() == DriverStation.Alliance.Blue)
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName)
                : LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);

        if (estimate == null)
            return new VisionOutput[0];

        VisionOutput output = new VisionOutput();
        output.latency = estimate.latency / 1000;
        output.timestamp = estimate.timestampSeconds;
        output.cameraName = cameraName;

        if (estimate.tagCount != 0) {
            output.amountOfTargets = estimate.tagCount;
            output.hasTargets = true;

            targets = estimate.rawFiducials;
            if (this.tags == null)
                fillTagsMap();
            if (this.tags == null)
                return new VisionOutput[0];
            analyze(output);

            output.robotPose = estimate.pose;
        }

        output.robotPoseMegaTag1 = estimateMegaTag1.pose;
        output.closestTargetDistMegaTag1 = estimateMegaTag1.avgTagDist;
        output.hasTargetsMegaTag1 = estimateMegaTag1.tagCount > 0;

        outputs.add(output);
        return outputs.toArray(new VisionOutput[0]);
    }

    private void analyze(VisionOutput output) {
        LimelightHelpers.RawFiducial closest = null;

        double minDist = Double.MAX_VALUE;
        double maxDist = -1;

        List<AprilTag> validTags = new ArrayList<>();
        List<Transform3d> transforms = new ArrayList<>();

        for (LimelightHelpers.RawFiducial target : targets) {
            if (!tags.containsKey(target.id))
                continue;

            AprilTag tag = tags.get(target.id);
            validTags.add(tag);
            transforms.add(new Transform3d());

            double dist = target.distToCamera;
            if (dist < minDist) {
                minDist = dist;
                closest = target;
            }
            if (dist > maxDist) {
                maxDist = dist;
            }
        }

        output.targetsIds = validTags.stream().mapToInt(x -> x.ID).toArray();
        output.targetsPoses = validTags.stream().map(x -> x.pose).toArray(Pose3d[]::new);
        output.cameraToTargetsTransforms = transforms.toArray(new Transform3d[0]);

        if (closest != null) {
            output.closestTargetDist = minDist;
            output.closestTargetId = closest.id;
            output.closestTargetPose = tags.get(closest.id).pose;
            output.cameraToClosestTargetTransform = new Transform3d();
        }

        output.ambiguity = 0;
        output.farthestTargetDist = maxDist;
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
        // Dynamically update filter list in Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride(
            cameraName,
            tags.keySet().stream().mapToInt(Integer::intValue).toArray()
        );
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
        // Dynamically update filter list in Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride(
            cameraName,
            tags.keySet().stream().mapToInt(Integer::intValue).toArray()
        );
    }

    private void fillTagsMap() {
        if (constants.fieldLayoutGetter.getFieldLayout(ignoredTags).isEmpty())
            return;
        
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).get().getTags())
            tags.put(tag.ID, tag);
    }
}
