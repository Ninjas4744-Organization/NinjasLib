package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Configuration for a {@link Vision} instance: which cameras to create, where they are mounted,
 * how to obtain the AprilTag field layout, and how to read the robot's current pose. Built with a
 * fluent {@code withX(...)} API and passed to {@link Vision#Vision(VisionConstants)}.
 * <p>
 * {@link Vision#Vision(VisionConstants)} treats a config with no cameras, no
 * {@link #fieldLayoutGetter}, or no {@link #robotPoseSupplier} as incomplete and disables vision
 * entirely, so all three must be set for vision to function.
 */
public class VisionConstants {
    /**
     * Cameras to construct, keyed by name, each paired with its robot-relative mounting
     * {@link Transform3d} and its {@link CameraType}. Populated via {@link #withLimelight(String)}
     * and {@link #withPhotonVision(String, Transform3d)}.
     */
    public Map<String, Pair<Transform3d, CameraType>> cameras = new HashMap<>();
    /** Supplies the AprilTag field layout used for pose estimation. Set via {@link #withFieldLayoutGetter(FieldLayoutGetter)}. */
    public FieldLayoutGetter fieldLayoutGetter = null;
    /** Supplies the robot's current estimated pose, used to drive the simulated vision system. Set via {@link #withRobotPoseSupplier(Supplier)}. */
    public Supplier<Pose2d> robotPoseSupplier = null;

    /**
     * Registers a Limelight camera. Limelight cameras compute their own pose estimate on-device,
     * so no mounting transform is needed here.
     *
     * @param name the Limelight's configured network name, used to look it up over NetworkTables
     * @return this, for chaining
     */
    public VisionConstants withLimelight(String name) {
        cameras.put(name, new Pair<>(new Transform3d(), CameraType.Limelight));
        return this;
    }

    /**
     * Registers a PhotonVision camera.
     *
     * @param name      the camera's configured name, used to look it up via {@code PhotonCamera}
     * @param transform the camera's position and orientation relative to the robot's center
     *                  (positive x forward, positive y left, counterclockwise-positive angle)
     * @return this, for chaining
     */
    public VisionConstants withPhotonVision(String name, Transform3d transform) {
        cameras.put(name, new Pair<>(transform, CameraType.PhotonVision));
        return this;
    }

    /**
     * Sets the source of the AprilTag field layout used for pose estimation.
     *
     * @param fieldLayoutGetter the field layout getter
     * @return this, for chaining
     */
    public VisionConstants withFieldLayoutGetter(FieldLayoutGetter fieldLayoutGetter) {
        this.fieldLayoutGetter = fieldLayoutGetter;
        return this;
    }

    /**
     * Sets the supplier used to obtain the robot's current pose, needed to keep the simulated
     * vision system in sync with the rest of the robot's simulation.
     *
     * @param robotPoseSupplier the robot pose supplier
     * @return this, for chaining
     */
    public VisionConstants withRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        return this;
    }

    /** The vision hardware/software backend a camera uses. */
    public enum CameraType{
        /** A coprocessor camera running PhotonVision. */
        PhotonVision,
        /** A Limelight smart camera. */
        Limelight
    }
}
