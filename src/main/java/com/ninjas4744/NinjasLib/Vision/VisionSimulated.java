package com.ninjas4744.NinjasLib.Vision;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;
import java.util.function.Supplier;

public class VisionSimulated extends VisionIO {
	private final VisionSystemSim _visionSystemSim = new VisionSystemSim("main");
	private final Supplier<Pose2d> _robotPoseSupplier;

    protected VisionSimulated(VisionConstants constants) {
		super(constants);

		_robotPoseSupplier = constants.simulationConstants.robotPoseSupplier;

		_visionSystemSim.addAprilTags(constants.fieldLayoutGetter.getFieldLayout(List.of()));

        PhotonCameraSim[] _simulatedCameras = new PhotonCameraSim[_cameras.length];
        SimCameraProperties[] _cameraProperties = new SimCameraProperties[_cameras.length];

		for (int i = 0; i < _cameras.length; i++) {
			_cameraProperties[i] = new SimCameraProperties();
			_cameraProperties[i].setCalibration(
					constants.simulationConstants.resolutionWidth,
					constants.simulationConstants.resolutionHeight,
					Rotation2d.fromDegrees(constants.simulationConstants.FOV));
			_cameraProperties[i].setCalibError(
					constants.simulationConstants.averageError, constants.simulationConstants.errorStdDev);
			_cameraProperties[i].setFPS(constants.simulationConstants.FPS);
			_cameraProperties[i].setAvgLatencyMs(constants.simulationConstants.averageLatency);
			_cameraProperties[i].setLatencyStdDevMs(constants.simulationConstants.latencyStdDev);

			_simulatedCameras[i] = new PhotonCameraSim(_cameras[i].getCamera(), _cameraProperties[i]);

			_visionSystemSim.addCamera(_simulatedCameras[i], constants.cameras.get(_cameras[i].getName()));
		}
	}

	@Override
	public void periodic() {
		super.periodic();
		_visionSystemSim.update(_robotPoseSupplier.get());
	}
}
