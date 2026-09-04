package frc.lib.NinjasLib.swerve.constants;

/**
 * Per-module hardware configuration for a single swerve module (motor CAN IDs, inversions and
 * CANCoder calibration), consumed by {@link SwerveModuleIOReal} and {@link SwerveModuleIOSim}.
 */
public class SwerveModuleConstants {
    /** Index of this module within the swerve drive (e.g. front-left, front-right, ...). */
    public int moduleNumber;

    /** CAN ID of the drive motor. */
    public int driveMotorID;

    /** CAN ID of the steer motor. */
    public int steerMotorID;

    /** Whether the drive motor is inverted. */
    public boolean driveMotorInverted;

    /** Whether the steer motor is inverted. */
    public boolean steerMotorInverted;

    /** CAN ID of the module's CANCoder absolute encoder. */
    public int canCoderID;

    /** Whether the CANCoder's sensor direction is inverted. */
    public boolean invertCANCoder;

    /** Magnet offset applied to the CANCoder reading, in rotations. */
    public double CANCoderOffset;

    /**
     * @param moduleNumber index of this module within the swerve drive
     * @param driveMotorID CAN ID of the drive motor
     * @param steerMotorID CAN ID of the steer motor
     * @param driveMotorInverted whether the drive motor is inverted
     * @param steerMotorInverted whether the steer motor is inverted
     * @param canCoderID CAN ID of the module's CANCoder
     * @param invertCANCoder whether the CANCoder's sensor direction is inverted
     * @param CANCoderOffset magnet offset applied to the CANCoder reading, in rotations
     */
    public SwerveModuleConstants(int moduleNumber,
                                 int driveMotorID,
                                 int steerMotorID,
                                 boolean driveMotorInverted,
                                 boolean steerMotorInverted,
                                 int canCoderID,
                                 boolean invertCANCoder,
                                 double CANCoderOffset) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.driveMotorInverted = driveMotorInverted;
        this.steerMotorInverted = steerMotorInverted;
        this.canCoderID = canCoderID;
        this.invertCANCoder = invertCANCoder;
        this.CANCoderOffset = CANCoderOffset;
    }
}
