package frc.lib.NinjasLib.localization.vision;

public interface VisionCameraIO {
    VisionOutput[] update();

    void ignoreTag(int id);

    void unIgnoreTag(int id);
}
