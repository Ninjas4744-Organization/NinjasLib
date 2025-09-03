package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

public class SwerveInput extends ChassisSpeeds implements StructSerializable {
    private boolean fieldRelative;

    public SwerveInput() {
        super();
        fieldRelative = false;
    }

    public SwerveInput(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldRelative) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.fieldRelative = fieldRelative;
    }

    public SwerveInput(ChassisSpeeds speeds, boolean fieldRelative) {
        vxMetersPerSecond = speeds.vxMetersPerSecond;
        vyMetersPerSecond = speeds.vyMetersPerSecond;
        omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
        this.fieldRelative = fieldRelative;
    }

    public Translation2d toTranslation() {
        return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
    }

    public double getSpeed() {
        return toTranslation().getNorm();
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public double getVx() {
        return vxMetersPerSecond;
    }

    public double getVy() {
        return vyMetersPerSecond;
    }

    public double getO() {
        return omegaRadiansPerSecond;
    }

    public SwerveInput getAsFieldRelative(Rotation2d robotAngle) {
        if (!fieldRelative)
            return new SwerveInput(ChassisSpeeds.fromRobotRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), true);
        return new SwerveInput(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    public SwerveInput getAsRobotRelative(Rotation2d robotAngle) {
        if (fieldRelative)
            return new SwerveInput(ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), false);
        return new SwerveInput(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
    }

    @Override
    public boolean equals(Object o) {
        return o == this
            || o instanceof SwerveInput i
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

    public static final SwerveInputStruct struct = new SwerveInputStruct();

    public static class SwerveInputStruct implements Struct<SwerveInput> {
        @Override
        public Class<SwerveInput> getTypeClass() {
            return SwerveInput.class;
        }

        @Override
        public String getTypeName() {
            return "SwerveInput";
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
        public SwerveInput unpack(ByteBuffer bb) {
            double vx = bb.getDouble();
            double vy = bb.getDouble();
            double omega = bb.getDouble();
            boolean fieldRelative = bb.get() != 0;
            return new SwerveInput(vx, vy, omega, fieldRelative);
        }

        @Override
        public void pack(ByteBuffer bb, SwerveInput value) {
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