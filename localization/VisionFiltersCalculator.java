package frc.lib.NinjasLib.localization;

import frc.lib.NinjasLib.localization.vision.VisionOutput;

/**
 * A pluggable sanity check for vision measurements, injected into {@link RobotPose} to decide which
 * camera results are trustworthy enough to correct the pose estimate with. This runs before {@link
 * VisionStrengthCalculator}: a measurement that fails the filter is discarded entirely, regardless of
 * how it would have been weighted.
 */
@FunctionalInterface
public interface VisionFiltersCalculator {
    /**
     * Decides whether a vision measurement should be accepted (e.g. checking ambiguity, target
     * distance, or plausibility against the current pose) before it can correct the robot's pose.
     *
     * @param visionOutput The camera's vision estimate for this cycle.
     * @return {@code true} if the measurement should be applied, {@code false} if it should be
     *     discarded.
     */
    boolean isPassed(VisionOutput visionOutput);
}
