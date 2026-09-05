package frc.lib.NinjasLib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.lib.NinjasLib.localization.RobotPose;

import java.nio.ByteBuffer;

/**
 * A {@link ChassisSpeeds} that additionally remembers whether it is field-relative or
 * robot-relative, so speeds can be passed around and converted between the two frames without
 * the caller needing to track that separately. This is the type {@link Swerve#drive} and
 * {@link SwerveController} pass speeds around as throughout the swerve subsystem.
 */
public class SwerveSpeeds extends ChassisSpeeds implements StructSerializable {
    /** Whether {@link #vxMetersPerSecond}/{@link #vyMetersPerSecond} are field-relative (vs. robot-relative). */
    public boolean fieldRelative;

    /** Zero speed, robot-relative. */
    public SwerveSpeeds() {
        super();
        fieldRelative = false;
    }

    /**
     * @param vxMetersPerSecond     x velocity, m/s
     * @param vyMetersPerSecond     y velocity, m/s
     * @param omegaRadiansPerSecond angular velocity, rad/s
     * @param fieldRelative         whether the x/y velocity is field-relative
     */
    public SwerveSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldRelative) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.fieldRelative = fieldRelative;
    }

    /** Wraps an existing {@link ChassisSpeeds}, tagging it as field- or robot-relative. */
    public SwerveSpeeds(ChassisSpeeds speeds, boolean fieldRelative) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    /** Builds speeds from an x/y velocity vector plus a separate angular velocity. */
    public SwerveSpeeds(Translation2d speeds, double omegaRadiansPerSecond, boolean fieldRelative) {
        this(speeds.getX(), speeds.getY(), omegaRadiansPerSecond, fieldRelative);
    }

    /** @return the x/y velocity as a {@link Translation2d} */
    public Translation2d toTranslation() {
        return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
    }

    /** @return the magnitude of the x/y velocity, m/s */
    public double getSpeed() {
        return toTranslation().getNorm();
    }

    /**
     * Converts these speeds to field-relative, using {@code robotAngle} to rotate them if they are
     * currently robot-relative. A no-op (aside from re-tagging) if already field-relative.
     *
     * @param robotAngle the robot's current heading, used only if a conversion is needed
     * @return an equivalent field-relative {@link SwerveSpeeds}
     */
    public SwerveSpeeds getAsFieldRelative(Rotation2d robotAngle) {
        if (!fieldRelative)
            return new SwerveSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), true);
        return new SwerveSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /** Same as {@link #getAsFieldRelative(Rotation2d)}, using the current robot heading from {@link RobotPose}. */
    public SwerveSpeeds getAsFieldRelative() {
        return getAsFieldRelative(RobotPose.get().getRobotPose().getRotation());
    }

    /**
     * Converts these speeds to robot-relative, using {@code robotAngle} to rotate them if they are
     * currently field-relative. A no-op (aside from re-tagging) if already robot-relative.
     *
     * @param robotAngle the robot's current heading, used only if a conversion is needed
     * @return an equivalent robot-relative {@link SwerveSpeeds}
     */
    public SwerveSpeeds getAsRobotRelative(Rotation2d robotAngle) {
        if (fieldRelative)
            return new SwerveSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle), false);
        return new SwerveSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, false);
    }

    /** Same as {@link #getAsRobotRelative(Rotation2d)}, using the current robot heading from {@link RobotPose}. */
    public SwerveSpeeds getAsRobotRelative() {
        return getAsRobotRelative(RobotPose.get().getRobotPose().getRotation());
    }

    /** Returns these speeds as field-relative or robot-relative depending on {@code fieldRelative}. */
    public SwerveSpeeds getAs(boolean fieldRelative, Rotation2d robotAngle) {
        if (fieldRelative)
            return getAsFieldRelative(robotAngle);
        return getAsRobotRelative(robotAngle);
    }

    /** Same as {@link #getAs(boolean, Rotation2d)}, using the current robot heading from {@link RobotPose}. */
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

    /** The shared {@link Struct} instance used to (de)serialize {@link SwerveSpeeds} over NetworkTables/logging. */
    public static final SwerveSpeedsStruct struct = new SwerveSpeedsStruct();

    /** WPILib {@link Struct} implementation that lets {@link SwerveSpeeds} be logged/sent as a raw struct type. */
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

        /** Reads a {@link SwerveSpeeds} from its packed binary struct representation. */
        @Override
        public SwerveSpeeds unpack(ByteBuffer bb) {
            double vx = bb.getDouble();
            double vy = bb.getDouble();
            double omega = bb.getDouble();
            boolean fieldRelative = bb.get() != 0;
            return new SwerveSpeeds(vx, vy, omega, fieldRelative);
        }

        /** Writes {@code value} to its packed binary struct representation. */
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