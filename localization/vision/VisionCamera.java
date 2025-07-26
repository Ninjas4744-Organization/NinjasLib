package frc.lib.NinjasLib.localization.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class VisionCamera<T> {
    protected List<VisionOutput> outputs;
    protected final List<Integer> ignoredTags;
    protected final VisionConstants constants;
    protected Map<Integer, AprilTag> tags;
    protected String cameraName;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public VisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        this.constants = constants;
        cameraName = name;
        outputs = new ArrayList<>();
        ignoredTags = new ArrayList<>();
        fillTagsMap();
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    public abstract List<VisionOutput> Update();

    /**
     * @return The camera processor that is being used by this VisionCamera
     */
    public abstract T getCamera();

    /**
     * @return name of the camera
     */
    public abstract String getName();

    /**
     * Adds an apriltag to the ignored apriltags list. If the camera sees a tag in the ignored list, it ignores it.
     * @param id the id of the apriltag to ignore
     */
    public void ignoreTag(int id) {
        ignoredTags.add(id);
        fillTagsMap();
    }

    private void fillTagsMap(){
        tags = new HashMap<>();
        for (AprilTag tag : constants.fieldLayoutGetter.getFieldLayout(ignoredTags).getTags())
            tags.put(tag.ID, tag);
    }
}
