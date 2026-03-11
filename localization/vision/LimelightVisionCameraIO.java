package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.swerve.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LimelightVisionCameraIO implements VisionCameraIO {
    private final String cameraName;
    private LimelightHelpers.RawFiducial[] targets;
    private final List<Integer> ignoredTags;
    private final VisionConstants constants;
    private Map<Integer, AprilTag> tags;

    public LimelightVisionCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        LimelightHelpers.SetIMUMode(cameraName, 0);
        ignoredTags = new ArrayList<>();
        fillTagsMap();
    }

    @Override
    public void updateInputs(VisionCameraIOInputsAutoLogged inputs) {
        inputs.outputs = new VisionOutput[0];
        List<VisionOutput> outputs = new ArrayList<>();

        Rotation2d robotYaw = Swerve.getInstance().getGyro().getYaw();
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            robotYaw = robotYaw.rotateBy(Rotation2d.k180deg);
        }
        LimelightHelpers.SetRobotOrientation(cameraName, robotYaw.getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraName);

        LimelightHelpers.PoseEstimate estimateMegaTag1 = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName)
                : LimelightHelpers.getBotPoseEstimate_wpiRed(cameraName);

        if (estimate == null)
            return;

        VisionOutput output = new VisionOutput();
        output.latency = estimate.latency / 1000;
        output.timestamp = estimate.timestampSeconds;
        output.cameraName = cameraName;

        if (estimate.tagCount != 0) {
            output.amountOfTargets = estimate.tagCount;
            output.hasTargets = true;

            targets = estimate.rawFiducials;
            analyze(output);

            output.robotPose = estimate.pose;
        }

        output.robotPoseMegaTag1 = estimateMegaTag1.pose;
        output.closestTargetDistMegaTag1 = estimateMegaTag1.avgTagDist;
        output.hasTargetsMegaTag1 = estimateMegaTag1.tagCount > 0;

        outputs.add(output);
        inputs.outputs = outputs.toArray(new VisionOutput[0]);
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
        // Dynamically update filter list in Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride(
            cameraName,
            tags.keySet().stream().mapToInt(Integer::intValue).toArray()
        );
        fillTagsMap();
    }

    private void fillTagsMap() {
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).getTags())
            tags.put(tag.ID, tag);
    }
}
