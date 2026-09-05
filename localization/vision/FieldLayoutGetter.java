package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.List;
import java.util.Optional;

/**
 * Supplies the {@link AprilTagFieldLayout} used for vision pose estimation, with the ability to
 * exclude specific tag IDs. Implementations typically wrap {@code AprilTagFieldLayout.loadField(...)}
 * for a given game field, optionally filtered down by {@link VisionCameraIO#ignoreTag(int)}.
 */
@FunctionalInterface
public interface FieldLayoutGetter {
    /**
     * Gets the field layout to use for tag localization.
     *
     * @param ignoredTags tag IDs that should be excluded from the returned layout
     * @return the field layout, or {@link Optional#empty()} if one is not currently available
     */
    Optional<AprilTagFieldLayout> getFieldLayout(List<Integer> ignoredTags);
}
