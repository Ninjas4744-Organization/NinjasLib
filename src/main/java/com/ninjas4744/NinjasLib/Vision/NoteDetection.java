package com.ninjas4744.NinjasLib.Vision;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants;
import com.ninjas4744.NinjasLib.RobotStateIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NoteDetection {
	private static final StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("NotePose", Pose2d.struct)
			.publish();

	private static VisionConstants.NoteDetectionConstants _constants;

	public static void setConstants(VisionConstants.NoteDetectionConstants constants){
		_constants = constants;
	}

	/**
	 * @return The pose of the note on the field
	 */
	public static Pose2d getNotePose() {
		if(_constants == null)
			throw new RuntimeException("Note detection constants not set. Please set them with setConstants(NoteDetectionConstants)");

		double tx = LimelightHelpers.getTX(_constants.limelightName);
		double ty = LimelightHelpers.getTY(_constants.limelightName);

		Translation2d translation = convertMeasurement(tx, ty);

		SmartDashboard.putNumber("Note Dist X", translation.getX());
		SmartDashboard.putNumber("Note Dist Y", translation.getY());

		Transform2d noteTransform =
				new Transform2d(new Translation2d(translation.getY(), -translation.getX()), new Rotation2d());

		Pose2d notePose = _constants.robotPoseSupplier.get().plus(noteTransform);
		publisher.set(notePose);

		return notePose;
	}

	/** Converts limelight's degrees measurement to meters */
	private static Translation2d convertMeasurement(double tx, double ty) {
		double distY = (_constants.limelightHeight - _constants.noteHeight)
				* Math.tan(Rotation2d.fromDegrees(_constants.limelightMountAngleX + ty)
						.getRadians());

		double distX = distY
				* Math.tan(Rotation2d.fromDegrees(_constants.limelightMountAngleY + tx)
						.getRadians());

		return new Translation2d(distX, distY);
	}

	/**
	 * @return Whether the limelight has detected a note
	 */
	public static boolean hasTarget() {
		if(_constants == null)
			throw new RuntimeException("Note detection constants not set. Please set them with setConstants(NoteDetectionConstants)");

		SmartDashboard.putBoolean("Limelight detected", LimelightHelpers.getTA(_constants.limelightName) > 0.5);
		return LimelightHelpers.getTA(_constants.limelightName) > 0.5;
	}
}
