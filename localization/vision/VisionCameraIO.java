package frc.lib.NinjasLib.localization.vision;

/**
 * IO layer for a single vision camera. Implementations wrap a specific vision system -
 * {@link LimelightVisionCameraIO} for a real Limelight, {@link PhotonVisionCameraIO} for a real
 * PhotonVision coprocessor camera, and {@link PhotonVisionSimCameraIO} for a simulated PhotonVision
 * camera - behind a common interface so {@link Vision} can aggregate any mix of them.
 */
public interface VisionCameraIO {
    /**
     * Polls the camera for new results and turns them into one {@link VisionOutput} per detected
     * pose estimate. Should be called once per robot loop (via {@link Vision#periodic()}).
     *
     * @return the pose estimate(s) produced since the last call, or an empty array if none are
     * available (e.g. no new results, no targets, or the camera is disconnected)
     */
    VisionOutput[] update();

    /**
     * Excludes an AprilTag from pose estimation. Once ignored, the camera will not use this tag
     * when computing robot pose estimates, even if it is visible.
     *
     * @param id the id of the apriltag to ignore
     */
    void ignoreTag(int id);

    /**
     * Reverses {@link #ignoreTag(int)}, allowing a previously ignored tag to be used again.
     *
     * @param id the id of the apriltag to stop ignoring
     */
    void unIgnoreTag(int id);
}
