package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.List;
import java.util.Optional;

@FunctionalInterface
public interface FieldLayoutGetter {
    Optional<AprilTagFieldLayout> getFieldLayout(List<Integer> ignoredTags);
}
