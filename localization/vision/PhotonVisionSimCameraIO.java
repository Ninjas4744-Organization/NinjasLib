package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/**
 * {@link VisionCameraIO} implementation for a simulated PhotonVision camera, used when
 * {@code Robot.isSimulation()} is true. Extends {@link PhotonVisionCameraIO} - so pose estimation
 * and target processing work exactly as they do on real hardware - but additionally builds a
 * {@code PhotonCameraSim} with representative camera properties (resolution, FOV, calibration
 * error, FPS and latency) so {@link Vision} can register it with a {@code VisionSystemSim}.
 */
public class PhotonVisionSimCameraIO extends PhotonVisionCameraIO {
    private PhotonCameraSim sim;

    /**
     * Creates a simulated PhotonVision camera, configuring simulated camera properties (a
     * 1280x720, 74-degree-FOV camera at 25 FPS with representative calibration error and latency).
     *
     * @param name       name of the camera.
     * @param cameraPose location of the camera on the robot (from center, positive x forward,
     *                   positive y left, and positive angle is counterclockwise).
     * @param constants  shared vision configuration, used to obtain the AprilTag field layout
     */
    public PhotonVisionSimCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(74));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(25);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        sim = new PhotonCameraSim(camera, cameraProp);
    }

    /**
     * @return the underlying {@code PhotonCameraSim}, used by {@link Vision} to register this
     * camera with the shared {@code VisionSystemSim}
     */
    public PhotonCameraSim getSim() {
        return sim;
    }
}
