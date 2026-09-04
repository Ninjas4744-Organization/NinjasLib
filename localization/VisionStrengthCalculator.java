package frc.lib.NinjasLib.localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.NinjasLib.localization.vision.VisionOutput;

import java.util.function.DoubleSupplier;

@FunctionalInterface
public interface VisionStrengthCalculator {
    Matrix<N3, N1> calculate(VisionOutput visionOutput);

    VisionStrengthCalculator kDefault = estimation -> VecBuilder.fill(0.05, 0.05, 0.025);

    static VisionStrengthCalculator ninjasFunction(double factor, double epsilon, double rotationFactor, DoubleSupplier odometryDrift) {
        return estimation -> {
            double distancePart = Math.pow(estimation.closestTargetDist, 2);

            double driftPart = Math.max(Math.pow(odometryDrift.getAsDouble() + epsilon, 0.5), Math.pow(odometryDrift.getAsDouble() + epsilon, 2));

            double visionStrength = 1 / (1 + distancePart / factor / driftPart);

            return VecBuilder.fill(visionStrength, visionStrength, visionStrength * rotationFactor);
        };
    }

    static VisionStrengthCalculator ninjasFunction(DoubleSupplier odometryDrift) {
        return ninjasFunction(1.5, 0.05, 0.5, odometryDrift);
    }
}
