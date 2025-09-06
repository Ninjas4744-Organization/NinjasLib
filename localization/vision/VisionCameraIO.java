package frc.lib.NinjasLib.localization.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionCameraIO {
    @AutoLog
    class VisionCameraIOInputs {
        public VisionOutput[] outputs;
    }

    default void updateInputs(VisionCameraIOInputsAutoLogged inputs) {
    }

    default void ignoreTag(int id) {
    }
}
