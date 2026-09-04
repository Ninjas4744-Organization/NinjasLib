package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

/**
 * A single pose estimate and its supporting AprilTag detections, produced by one call to
 * {@link VisionCameraIO#update()}. This is the common data type all camera IO implementations
 * ({@link LimelightVisionCameraIO}, {@link PhotonVisionCameraIO}, {@link PhotonVisionSimCameraIO})
 * report through, and what {@link Vision} consumes and republishes to NetworkTables.
 * <p>
 * Implements {@link StructSerializable} via the nested {@link VisionOutputStruct} so instances can
 * be logged/published directly.
 */
public class VisionOutput implements StructSerializable {
	/** The pose of the robot. If using limelight, this will be MegaTag2 */
	public Pose2d robotPose = new Pose2d();

	/** The pose of the robot from MegaTag1. Only works with limelight */
	public Pose2d robotPoseMegaTag1 = new Pose2d();

	/** The time at which the pose was detected */
	public double timestamp = 0;

    /** The time between image capture and publish to NT in seconds */
    public double latency = 0;

	/**
	 * The id of the tag which was detected the closest to the camera
	 */
	public int closestTargetId = 0;

	/**
	 * The pose of the tag which was detected the closest to the camera
	 */
	public Pose3d closestTargetPose = new Pose3d();

	/**
	 * The ids of the detected tags
	 */
	public int[] targetsIds = new int[0];

	/**
	 * The poses of the detected tags
	 */
	public Pose3d[] targetsPoses = new Pose3d[0];

	/**
	 * The transformation between the camera and the closest tag
	 */
	public Transform3d cameraToClosestTargetTransform = new Transform3d();

	/**
	 * The transformation between the camera and all the detected tags
	 */
	public Transform3d[] cameraToTargetsTransforms = new Transform3d[0];

	/**
	 * The pose ambiguity of the detected target(s); lower is more confident. Only meaningfully
	 * populated by {@link PhotonVisionCameraIO} - {@link LimelightVisionCameraIO} always reports 0.
	 */
	public double ambiguity = 0;

	/** The distance from the camera of the tag which was detected the farthest */
	public double farthestTargetDist = 0;

	/** The distance from the camera of the tag which was detected the closest */
	public double closestTargetDist = 0;

	/** The distance from the camera of the tag which was detected the closest mega tag 1 */
	public double closestTargetDistMegaTag1 = 0;

	/** Whether the camera detected any tags */
	public boolean hasTargets = false;

	/** Whether the camera detected any tags mega tag 1 */
	public boolean hasTargetsMegaTag1 = false;

	/** How many targets the camera detected */
	public int amountOfTargets = 0;

	/** The name of the camera */
	public String cameraName = "";

	/** The shared {@link Struct} instance used to serialize {@link VisionOutput} for logging/NetworkTables. */
	public static final VisionOutputStruct struct = new VisionOutputStruct();

	/**
	 * WPILib {@link Struct} implementation that (de)serializes a {@link VisionOutput} to/from raw
	 * bytes. The {@code targetsIds}, {@code targetsPoses} and {@code cameraToTargetsTransforms}
	 * arrays are packed/unpacked with a fixed capacity of 3 entries, padded with sentinel values
	 * ({@code -1} for ids, {@code -999} coordinates for poses/transforms) when fewer targets are
	 * present.
	 */
	public static class VisionOutputStruct implements Struct<VisionOutput> {

		/** @return {@link VisionOutput}, the type this struct (de)serializes */
		@Override
		public Class<VisionOutput> getTypeClass() {
			return VisionOutput.class;
		}

		/** @return the struct's schema type name, {@code "VisionOutput"} */
		@Override
		public String getTypeName() {
			return "VisionOutput";
		}

		/** @return the fixed serialized size in bytes of a packed {@link VisionOutput} */
		@Override
		public int getSize() {
			int size = 0;
			size += Pose2d.struct.getSize(); // robotPose
			size += Pose2d.struct.getSize(); // robotPoseMegaTag1
			size += kSizeDouble; // timestamp
			size += kSizeDouble; // latency
			size += kSizeDouble; // closestTargetId (was int)
			size += Pose3d.struct.getSize(); // closestTargetPose

			size += 3 * kSizeDouble; // targetsIds array only 3
			size += 3 * Pose3d.struct.getSize(); // targetsPoses array only 3
			size += Transform3d.struct.getSize(); // cameraToClosestTargetTransform
			size += 3 * Transform3d.struct.getSize(); // cameraToTargetsTransforms, max 3

			size += kSizeDouble * 4; // ambiguity, farthestTargetDist, closestTargetDist, closestTargetDistMegaTag1
			size += kSizeBool * 2; // hasTargets
			size += kSizeDouble; // amountOfTargets (was int, now double)

			return size;
		}

		/** @return the nested structs referenced by {@link VisionOutput}'s fields */
		@Override
		public Struct<?>[] getNested() {
			return new Struct<?>[]{
				Pose2d.struct,          // robotPose and robotPoseMegaTag1
				Pose3d.struct,          // closestTargetPose
				Transform3d.struct      // cameraToClosestTargetTransform and cameraToTargetsTransforms
			};
		}

		/** @return the struct's binary schema string describing field layout and order */
		@Override
		public String getSchema() {
			return "Pose2d robotPose;Pose2d robotPoseMegaTag1;double timestamp;double latency;double closestTargetId;Pose3d closestTargetPose;" +
				"double target1Id;double target2Id;double target3Id;Pose3d target1Pose;Pose3d target2Pose;Pose3d target3Pose;" +
				"Transform3d cameraToClosestTargetTransform;Transform3d cameraToTarget1Transform;Transform3d cameraToTarget2Transform;Transform3d cameraToTarget3Transform;" +
				"double ambiguity;double farthestTargetDist;double closestTargetDist;double closestTargetDistMegaTag1;bool hasTargets;bool hasTargetsMegaTag1;double amountOfTargets";
		}

		/**
		 * Reads a {@link VisionOutput} from its packed binary representation, reconstructing the
		 * variable-length target arrays from their fixed-capacity, sentinel-padded encoding.
		 *
		 * @param bb the buffer to read from
		 * @return the decoded {@link VisionOutput}
		 */
		@Override
		public VisionOutput unpack(ByteBuffer bb) {
			VisionOutput output = new VisionOutput();

			output.robotPose = Pose2d.struct.unpack(bb);
			output.robotPoseMegaTag1 = Pose2d.struct.unpack(bb);
			output.timestamp = bb.getDouble();
			output.latency = bb.getDouble();

			output.closestTargetId = (int) bb.getDouble();
			output.closestTargetPose = Pose3d.struct.unpack(bb);

			int target2 = (int) bb.getDouble();
			int target1 = (int) bb.getDouble();
			int target3 = (int) bb.getDouble();
			int targetsLength = (target1 != -1 ? 1 : 0) + (target2 != -1 ? 1 : 0) + (target3 != -1 ? 1 : 0);
			output.targetsIds = new int[targetsLength];
			int idx = 0;
			if (target1 != -1) output.targetsIds[idx++] = target1;
			if (target2 != -1) output.targetsIds[idx++] = target2;
			if (target3 != -1) output.targetsIds[idx++] = target3;

			Pose3d pose1 = Pose3d.struct.unpack(bb);
			Pose3d pose2 = Pose3d.struct.unpack(bb);
			Pose3d pose3 = Pose3d.struct.unpack(bb);
			int posesLength = (pose1.getX() != -999 ? 1 : 0) + (pose2.getX() != -999 ? 1 : 0) + (pose3.getX() != -999 ? 1 : 0);
			output.targetsPoses = new Pose3d[posesLength];
			idx = 0;
			if (pose1.getX() != -999) output.targetsPoses[idx++] = pose1;
			if (pose2.getX() != -999) output.targetsPoses[idx++] = pose2;
			if (pose3.getX() != -999) output.targetsPoses[idx++] = pose3;

			output.cameraToClosestTargetTransform = Transform3d.struct.unpack(bb);

			Transform3d transform1 = Transform3d.struct.unpack(bb);
			Transform3d transform2 = Transform3d.struct.unpack(bb);
			Transform3d transform3 = Transform3d.struct.unpack(bb);
			int transformsLength = (transform1.getX() != -999 ? 1 : 0) + (transform2.getX() != -999 ? 1 : 0) + (transform3.getX() != -999 ? 1 : 0);
			output.cameraToTargetsTransforms = new Transform3d[transformsLength];
			idx = 0;
			if (transform1.getX() != -999) output.cameraToTargetsTransforms[idx++] = transform1;
			if (transform2.getX() != -999) output.cameraToTargetsTransforms[idx++] = transform2;
			if (transform3.getX() != -999) output.cameraToTargetsTransforms[idx++] = transform3;

			output.ambiguity = bb.getDouble();
			output.farthestTargetDist = bb.getDouble();
			output.closestTargetDist = bb.getDouble();
			output.closestTargetDistMegaTag1 = bb.getDouble();

			output.hasTargets = bb.get() != 0;
			output.hasTargetsMegaTag1 = bb.get() != 0;
			output.amountOfTargets = (int) bb.getDouble();

			return output;
		}

		/**
		 * Writes a {@link VisionOutput} to its packed binary representation. The variable-length
		 * target arrays are truncated/padded to a fixed capacity of 3 entries with sentinel values.
		 *
		 * @param bb    the buffer to write to
		 * @param value the {@link VisionOutput} to encode
		 */
		@Override
		public void pack(ByteBuffer bb, VisionOutput value) {
			Pose2d.struct.pack(bb, value.robotPose);
			Pose2d.struct.pack(bb, value.robotPoseMegaTag1);
			bb.putDouble(value.timestamp);
			bb.putDouble(value.latency);

			bb.putDouble(value.closestTargetId);
			Pose3d.struct.pack(bb, value.closestTargetPose);

			for (int i = 0; i < 3; i++) {
				if (i < value.targetsIds.length)
					bb.putDouble(value.targetsIds[i]);
				else
					bb.putDouble(-1);
			}

			for (int i = 0; i < 3; i++) {
				if (i < value.targetsPoses.length)
					Pose3d.struct.pack(bb, value.targetsPoses[i]);
				else
					Pose3d.struct.pack(bb, new Pose3d(-999, -999, -999, Rotation3d.kZero));
			}

			Transform3d.struct.pack(bb, value.cameraToClosestTargetTransform);

			for (int i = 0; i < 3; i++) {
				if (i < value.cameraToTargetsTransforms.length)
					Transform3d.struct.pack(bb, value.cameraToTargetsTransforms[i]);
				else
					Transform3d.struct.pack(bb, new Transform3d(-999, -999, -999, Rotation3d.kZero));
			}

			bb.putDouble(value.ambiguity);
			bb.putDouble(value.farthestTargetDist);
			bb.putDouble(value.closestTargetDist);
			bb.putDouble(value.closestTargetDistMegaTag1);

			bb.put((byte) (value.hasTargets ? 1 : 0));
			bb.put((byte) (value.hasTargetsMegaTag1 ? 1 : 0));
			bb.putDouble(value.amountOfTargets);
		}

		/** @return {@code false} - {@link VisionOutput} is a mutable, field-based value holder */
		@Override
		public boolean isImmutable() {
			return false;
		}
	}
}
