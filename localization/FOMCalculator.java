package frc.lib.NinjasLib.localization;

import frc.lib.NinjasLib.localization.vision.VisionOutput;

@FunctionalInterface
public interface FOMCalculator {
    double[] calculateFOM(VisionOutput estimation);
}
