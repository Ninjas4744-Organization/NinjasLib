package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.List;

@FunctionalInterface
public interface FieldLayoutGetter {
    AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags);
}
