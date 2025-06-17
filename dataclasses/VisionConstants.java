package frc.lib.NinjasLib.dataclasses;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Map;
import java.util.function.Supplier;

public class VisionConstants {
    public Map<String, Pair<Transform3d, CameraType>> cameras;
    public double maxAmbiguity;
    public double maxDistance;
    public FieldLayoutGetter fieldLayoutGetter;
    public SimulationConstants simulationConstants;

    public enum CameraType{
        PhotonVision,
        Limelight
    }

    public static class SimulationConstants {
        public int resolutionWidth;
        public int resolutionHeight;
        public double FOV;
        public double averageError;
        public double errorStdDev;
        public int FPS;
        public int averageLatency;
        public int latencyStdDev;
        public Supplier<Pose2d> robotPoseSupplier;
    }
}
