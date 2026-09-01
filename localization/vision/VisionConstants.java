package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Map;
import java.util.function.Supplier;

public class VisionConstants {
    public Map<String, Pair<Transform3d, CameraType>> cameras;
    public FieldLayoutGetter fieldLayoutGetter;
    public Supplier<Pose2d> robotPoseSupplier;

    public enum CameraType{
        PhotonVision,
        Limelight
    }
}
