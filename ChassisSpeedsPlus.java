package frc.lib.NinjasLib;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class ChassisSpeedsPlus extends ChassisSpeeds {
    public Translation2d toTranslation() {
        return new Translation2d(vxMetersPerSecond, vyMetersPerSecond);
    }

    public double getSpeed() {
        return toTranslation().getNorm();
    }

    public ChassisSpeedsPlus(Translation2d vMetersPerSecond, double omegaRadiansPerSecond) {
        super(vMetersPerSecond.getX(), vMetersPerSecond.getY(), omegaRadiansPerSecond);
    }

    public ChassisSpeedsPlus() {
    }

    public ChassisSpeedsPlus(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    public ChassisSpeedsPlus(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
        super(vx, vy, omega);
    }
}
