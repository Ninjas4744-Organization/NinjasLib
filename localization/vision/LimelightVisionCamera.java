package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

import java.util.ArrayList;
import java.util.List;

public class LimelightVisionCamera extends VisionCamera<LimelightHelpers> {
    private final String cameraName;
    private LimelightHelpers.LimelightTarget_Fiducial[] targets;

    public LimelightVisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);
        this.cameraName = name;
        LimelightHelpers.SetIMUMode(cameraName, 0);
    }

    @Override
    public List<VisionOutput> Update() {
        outputs.clear();

        // Set the robot's yaw from the swerve pose estimator
        Rotation2d robotYaw = RobotStateWithSwerve.getInstance().getRobotPose().getRotation();
        if (RobotStateBase.getAlliance() == DriverStation.Alliance.Red) {
            robotYaw = robotYaw.unaryMinus();
        }
        LimelightHelpers.SetRobotOrientation(cameraName, robotYaw.getDegrees(), 0, 0, 0, 0, 0);

        // Get pose estimate from the Limelight (MegaTag2 mode)
        LimelightHelpers.PoseEstimate estimate = (RobotStateBase.getAlliance() == DriverStation.Alliance.Blue)
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(cameraName);

        if (estimate.tagCount == 0)
            return new ArrayList<>();

        // Populate VisionOutput
        VisionOutput output = new VisionOutput();
        output.cameraName = cameraName;
        output.amountOfTargets = estimate.tagCount;
        output.hasTargets = true;

        // Analyze tag info
        targets = LimelightHelpers.getLatestResults(cameraName).targets_Fiducials;
        analyze(output);

        // Only use if within range
        if (output.closestTargetDist < constants.maxDistance) {
            output.robotPose = estimate.pose;
            output.timestamp = estimate.timestampSeconds;
        } else {
            output.hasTargets = false;
            output.amountOfTargets = 0;
        }

        outputs.add(output);
        return outputs;
    }

    private void analyze(VisionOutput output) {
        LimelightHelpers.LimelightTarget_Fiducial closest = null;

        double minDist = Double.MAX_VALUE;
        double maxDist = -1;

        List<AprilTag> validTags = new ArrayList<>();
        List<Transform3d> transforms = new ArrayList<>();

        for (LimelightHelpers.LimelightTarget_Fiducial target : targets) {
            if (!tags.containsKey((int) target.fiducialID))
                continue;

            AprilTag tag = tags.get((int) target.fiducialID);
            validTags.add(tag);
            transforms.add(new Transform3d(new Pose3d(), target.getTargetPose_CameraSpace()));

            double dist = target.getCameraPose_TargetSpace().getTranslation().getNorm();
            if (dist < minDist) {
                minDist = dist;
                closest = target;
            }
            if (dist > maxDist) {
                maxDist = dist;
            }
        }

        output.targets = validTags.toArray(new AprilTag[0]);
        output.cameraToTargetsTransforms = transforms.toArray(new Transform3d[0]);

        if (closest != null) {
            output.closestTargetDist = minDist;
            output.closestTarget = tags.get((int) closest.fiducialID);
            output.cameraToClosestTargetTransform = new Transform3d(new Pose3d(), closest.getTargetPose_CameraSpace());
        }

        output.maxAmbiguity = 0; // MegaTag2 handles ambiguity internally
        output.farthestTargetDist = maxDist;
    }

    @Override
    public LimelightHelpers getCamera() {
        return new LimelightHelpers();
    }

    @Override
    public String getName() {
        return cameraName;
    }

    @Override
    public void ignoreTag(int id) {
        super.ignoreTag(id);
        // Dynamically update filter list in Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride(
            cameraName,
            tags.keySet().stream().mapToInt(Integer::intValue).toArray()
        );
    }
}
