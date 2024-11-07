package com.ninjas4744.NinjasLib.DataClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import com.ninjas4744.NinjasLib.Vision.FieldLayoutGetter;

import java.util.Map;
import java.util.function.Supplier;

public class VisionConstants {
    public Map<String, Transform3d> cameras;
    public double maxAmbiguity;
    public double maxDistance;
    public FieldLayoutGetter fieldLayoutGetter;
    public SimulationConstants simulationConstants;
    public NoteDetectionConstants noteDetectionConstants;

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

    public static class NoteDetectionConstants {
        public String limelightName;
        public double limelightMountAngleX;
        public double limelightMountAngleY;
        public double limelightHeight;
        public double noteHeight;
        public Supplier<Pose2d> robotPoseSupplier;
    }
}
