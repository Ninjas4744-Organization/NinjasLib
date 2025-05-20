package frc.lib.NinjasLib.DataClasses;

@FunctionalInterface
public interface FOMCalculator {
    double[] calculateFOM(VisionOutput estimation);
}
