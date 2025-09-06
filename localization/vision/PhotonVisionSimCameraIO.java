package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class PhotonVisionSimCameraIO extends PhotonVisionCameraIO {
    private PhotonCameraSim sim;

    /**
     * @param name       Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *                   positive y left, and positive angle is counterclockwise).
     * @param constants
     */
    public PhotonVisionSimCameraIO(String name, Transform3d cameraPose, VisionConstants constants) {
        super(name, cameraPose, constants);

        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(74));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(25);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(10);
        sim = new PhotonCameraSim(camera, cameraProp);
    }

    public PhotonCameraSim getSim() {
        return sim;
    }
}
