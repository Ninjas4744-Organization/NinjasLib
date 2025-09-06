package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

public class VisionOutput implements StructSerializable {
	/** The pose of the robot */
	public Pose2d robotPose = new Pose2d();

	/** The time at which the pose was detected */
	public double timestamp = 0;

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

	/** The ambiguity of the tag which was detected most ambiguously */
	public double maxAmbiguity = 0;

	/** The distance from the camera of the tag which was detected the farthest */
	public double farthestTargetDist = 0;

	/** The distance from the camera of the tag which was detected the closest */
	public double closestTargetDist = 0;

	/** Whether the camera detected any tags */
	public boolean hasTargets = false;

	/** How many targets the camera detected */
	public int amountOfTargets = 0;

	/**
	 * The name of the camera
	 */
	public String cameraName = "";

	public static final VisionOutputStruct struct = new VisionOutputStruct();

	public static class VisionOutputStruct implements Struct<VisionOutput> {

		@Override
		public Class<VisionOutput> getTypeClass() {
			return VisionOutput.class;
		}

		@Override
		public String getTypeName() {
			return "VisionOutput";
		}

		@Override
		public int getSize() {
			int size = 0;
			size += Pose2d.struct.getSize(); // robotPose
			size += kSizeDouble; // timestamp
			size += kSizeDouble; // closestTargetId (was int)
			size += Pose3d.struct.getSize(); // closestTargetPose

			size += 4 + 10 * kSizeDouble; // targetsIds array (was int[], now double[], max 10 elements)
			size += Transform3d.struct.getSize(); // cameraToClosestTargetTransform
			size += 4 + 10 * Transform3d.struct.getSize(); // cameraToTargetsTransforms, max 10

			size += kSizeDouble * 3; // maxAmbiguity, farthestTargetDist, closestTargetDist
			size += kSizeBool; // hasTargets
			size += kSizeDouble; // amountOfTargets (was int, now double)
			size += 4 + 32; // cameraName, max 32 bytes

			return size;
		}

		@Override
		public Struct<?>[] getNested() {
			return new Struct<?>[]{
				Pose2d.struct,          // robotPose
				Pose3d.struct,          // closestTargetPose
				Transform3d.struct      // cameraToClosestTargetTransform and cameraToTargetsTransforms
			};
		}

		@Override
		public String getSchema() {
//			return "Pose2d robotPose;double timestamp;int closestTargetId;Pose3d closestTargetPose;" +
//				"int[] targetsIds;Transform3d cameraToClosestTargetTransform;Transform3d[] cameraToTargetsTransforms;" +
//				"double maxAmbiguity;double farthestTargetDist;double closestTargetDist;" +
//				"bool hasTargets;int amountOfTargets;string cameraName";

			return "Pose2d robotPose;double timestamp;double closestTargetId;Pose3d closestTargetPose;" +
				"double[] targetsIds";
		}

		@Override
		public VisionOutput unpack(ByteBuffer bb) {
			VisionOutput output = new VisionOutput();

			output.robotPose = Pose2d.struct.unpack(bb);
			output.timestamp = bb.getDouble();

			output.closestTargetId = (int) bb.getDouble();
			output.closestTargetPose = Pose3d.struct.unpack(bb);

			// targetsIds array
			int targetsLength = bb.getInt();
			output.targetsIds = new int[targetsLength];
			for (int i = 0; i < targetsLength; i++) {
				output.targetsIds[i] = (int) bb.getDouble();
			}

			output.cameraToClosestTargetTransform = Transform3d.struct.unpack(bb);

			int transformsLength = bb.getInt();
			output.cameraToTargetsTransforms = new Transform3d[transformsLength];
			for (int i = 0; i < transformsLength; i++) {
				output.cameraToTargetsTransforms[i] = Transform3d.struct.unpack(bb);
			}

			output.maxAmbiguity = bb.getDouble();
			output.farthestTargetDist = bb.getDouble();
			output.closestTargetDist = bb.getDouble();

			output.hasTargets = bb.get() != 0;
			output.amountOfTargets = (int) bb.getDouble();

			// cameraName string
			int nameLength = bb.getInt();
			byte[] nameBytes = new byte[nameLength];
			bb.get(nameBytes);
			output.cameraName = new String(nameBytes);

			return output;
		}

		@Override
		public void pack(ByteBuffer bb, VisionOutput value) {
			Pose2d.struct.pack(bb, value.robotPose);
			bb.putDouble(value.timestamp);

			bb.putDouble(value.closestTargetId);
			Pose3d.struct.pack(bb, value.closestTargetPose);

			// targetsIds array
			bb.putInt(value.targetsIds.length);
			for (int id : value.targetsIds) {
				bb.putDouble(id);
			}

			Transform3d.struct.pack(bb, value.cameraToClosestTargetTransform);

			bb.putInt(value.cameraToTargetsTransforms.length);
			for (Transform3d t : value.cameraToTargetsTransforms) {
				Transform3d.struct.pack(bb, t);
			}

			bb.putDouble(value.maxAmbiguity);
			bb.putDouble(value.farthestTargetDist);
			bb.putDouble(value.closestTargetDist);

			bb.put((byte) (value.hasTargets ? 1 : 0));
			bb.putDouble(value.amountOfTargets);

			byte[] nameBytes = value.cameraName.getBytes();
			bb.putInt(nameBytes.length);
			bb.put(nameBytes);
		}

		@Override
		public boolean isImmutable() {
			return false;
		}
	}

}
