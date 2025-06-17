package frc.lib.NinjasLib.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.NinjasLib.dataclasses.VisionConstants;
import frc.lib.NinjasLib.dataclasses.VisionOutput;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class VisionCamera<T> {
    protected final VisionOutput _output;
    protected final List<Integer> _ignoredTags;
    protected final VisionConstants _constants;
    protected Map<Integer, AprilTag> _tags;

    /**
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward,
     *     positive y left, and positive angle is counterclockwise).
     */
    public VisionCamera(String name, Transform3d cameraPose, VisionConstants constants) {
        _constants = constants;

        _output = new VisionOutput();
        _output.cameraName = name;

        _ignoredTags = new ArrayList<>();
        fillTagsMap();
    }

    /**
     * Updates the results of this camera, should run on periodic
     * @return The vision output of this camera
     */
    public abstract VisionOutput Update();

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
        _ignoredTags.add(id);
        fillTagsMap();
    }

    private void fillTagsMap(){
        _tags = new HashMap<>();
        for(AprilTag tag : _constants.fieldLayoutGetter.getFieldLayout(_ignoredTags).getTags())
            _tags.put(tag.ID, tag);
    }
}
