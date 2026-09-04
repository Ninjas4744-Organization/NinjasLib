package frc.lib.NinjasLib.localization;

import frc.lib.NinjasLib.localization.vision.VisionOutput;

@FunctionalInterface
public interface VisionFiltersCalculator {
    boolean isPassed(VisionOutput visionOutput);
}
