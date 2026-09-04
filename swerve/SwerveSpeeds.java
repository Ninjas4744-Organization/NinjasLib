package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.lib.NinjasLib.localization.RobotPose;

import java.nio.ByteBuffer;

public class SwerveSpeeds extends ChassisSpeeds implements StructSerializable {
    public boolean fieldRelative;

    public SwerveSpeeds() {
        super();
        fieldRelative = false;
    }

    public SwerveSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldRelative) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.fieldRelative = fieldRelative;
    }

    public SwerveSpeeds(ChassisSpeeds speeds, boolean fieldRelative) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    public SwerveSpeeds(Translation2d speeds, double omegaRadiansPerSecond, boolean fieldRelative) {
        this(speeds.getX(), speeds.getY(), omegaRadiansPerSecond, fieldRelative);
    }

    public Translation2d toTranslation() {
        return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
    }

    public double getSpeed() {
        return toTranslation().getNorm();
    }

    public SwerveSpeeds getAsFieldRelative(Rotation2d robotAngle) {
        if (!fieldRelative)
            return new SwerveSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), true);
        return new SwerveSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    public SwerveSpeeds getAsFieldRelative() {
        return getAsFieldRelative(RobotPose.get().getRobotPose().getRotation());
    }

    public SwerveSpeeds getAsRobotRelative(Rotation2d robotAngle) {
        if (fieldRelative)
            return new SwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), false);
        return new SwerveSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
    }

    public SwerveSpeeds getAsRobotRelative() {
        return getAsRobotRelative(RobotPose.get().getRobotPose().getRotation());
    }

    public SwerveSpeeds getAs(boolean fieldRelative, Rotation2d robotAngle) {
        if (fieldRelative)
            return getAsFieldRelative(robotAngle);
        return getAsRobotRelative(robotAngle);
    }

    public SwerveSpeeds getAs(boolean fieldRelative) {
        if (fieldRelative)
            return getAsFieldRelative();
        return getAsRobotRelative();
    }

    @Override
    public boolean equals(Object o) {
        return o == this
            || o instanceof SwerveSpeeds i
            && vxMetersPerSecond == i.vxMetersPerSecond
            && vyMetersPerSecond == i.vyMetersPerSecond
            && omegaRadiansPerSecond == i.omegaRadiansPerSecond
            && fieldRelative == i.fieldRelative;
    }

    @Override
    public String toString() {
        return String.format(
            "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s, Field Relative: %b)",
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, fieldRelative);
    }

    public static final SwerveSpeedsStruct struct = new SwerveSpeedsStruct();

    public static class SwerveSpeedsStruct implements Struct<SwerveSpeeds> {
        @Override
        public Class<SwerveSpeeds> getTypeClass() {
            return SwerveSpeeds.class;
        }

        @Override
        public String getTypeName() {
            return "SwerveSpeeds";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3 + kSizeBool;
        }

        @Override
        public String getSchema() {
            return "double vxMetersPerSecond;double vyMetersPerSecond;double omegaRadiansPerSecond;bool fieldRelative";
        }

        @Override
        public SwerveSpeeds unpack(ByteBuffer bb) {
            double vx = bb.getDouble();
            double vy = bb.getDouble();
            double omega = bb.getDouble();
            boolean fieldRelative = bb.get() != 0;
            return new SwerveSpeeds(vx, vy, omega, fieldRelative);
        }

        @Override
        public void pack(ByteBuffer bb, SwerveSpeeds value) {
            bb.putDouble(value.vxMetersPerSecond);
            bb.putDouble(value.vyMetersPerSecond);
            bb.putDouble(value.omegaRadiansPerSecond);
            bb.put((byte) (value.fieldRelative ? 1 : 0));
        }

        @Override
        public boolean isImmutable() {
            return false;
        }
    }
}