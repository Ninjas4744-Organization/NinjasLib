package frc.lib.NinjasLib.dataclasses;

@FunctionalInterface
public interface FOMCalculator {
    double[] calculateFOM(VisionOutput estimation);
}
