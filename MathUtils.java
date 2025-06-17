package frc.lib.NinjasLib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class MathUtils {
    public static double dstLinePoint(Translation2d point, Translation2d line, Translation2d pointInLine) {
        // Vector from point on line to the external point
        Translation2d pointToPointInLine = point.minus(pointInLine);

        // Get the perpendicular distance using the cross product magnitude
        return Math.abs(pointToPointInLine.getX() * line.getY() - pointToPointInLine.getY() * line.getX()) / line.getNorm();
    }

    public static double dstPointsInAxis(Translation2d p1, Translation2d p2, Translation2d axis) {
        // Vector from p1 to p2
        Translation2d delta = p2.minus(p1);

        // Normalize the axis vector (so projection is properly scaled)
        Translation2d axisNormalized = axis.div(axis.getNorm());

        // Dot product gives scalar projection of delta onto axis
        return Math.abs(delta.getX() * axisNormalized.getX() + delta.getY() * axisNormalized.getY());
    }

    public static int clamp(int value, int low, int high) {
        return MathUtil.clamp(value, low, high);
    }

    public static double clamp(double value, double low, double high) {
        return MathUtil.clamp(value, low, high);
    }

    public static double applyDeadband(double value, double deadband, double maxMagnitude) {
        return MathUtil.applyDeadband(value, deadband, maxMagnitude);
    }

    public static double applyDeadband(double value, double deadband) {
        return MathUtil.applyDeadband(value, deadband);
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        return MathUtil.inputModulus(input, minimumInput, maximumInput);
    }

    public static double angleModulus(double angleRadians) {
        return MathUtil.angleModulus(angleRadians);
    }

    public static double interpolate(double startValue, double endValue, double t) {
        return MathUtil.interpolate(startValue, endValue, t);
    }

    public static double inverseInterpolate(double startValue, double endValue, double q) {
        return MathUtil.inverseInterpolate(startValue, endValue, q);
    }

    public static boolean isNear(double expected, double actual, double tolerance) {
        return MathUtil.isNear(expected, actual, tolerance);
    }

    public static boolean isNear(double expected, double actual, double tolerance, double min, double max) {
        return MathUtil.isNear(expected, actual, tolerance, min, max);
    }

    public static Translation2d slewRateLimit(Translation2d input, Translation2d previous, double rateLimit, double dt) {
        return MathUtil.slewRateLimit(input, previous, rateLimit, dt);
    }

    public static Translation3d slewRateLimit(Translation3d input, Translation3d previous, double rateLimit, double dt) {
        return MathUtil.slewRateLimit(input, previous, rateLimit, dt);
    }

}