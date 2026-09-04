package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class VisionConstants {
    public Map<String, Pair<Transform3d, CameraType>> cameras = new HashMap<>();
    public FieldLayoutGetter fieldLayoutGetter = null;
    public Supplier<Pose2d> robotPoseSupplier = null;

    public VisionConstants withLimelight(String name) {
        cameras.put(name, new Pair<>(new Transform3d(), CameraType.Limelight));
        return this;
    }

    public VisionConstants withPhotonVision(String name, Transform3d transform) {
        cameras.put(name, new Pair<>(transform, CameraType.PhotonVision));
        return this;
    }

    public VisionConstants withFieldLayoutGetter(FieldLayoutGetter fieldLayoutGetter) {
        this.fieldLayoutGetter = fieldLayoutGetter;
        return this;
    }

    public VisionConstants withRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        return this;
    }

    public enum CameraType{
        PhotonVision,
        Limelight
    }
}
