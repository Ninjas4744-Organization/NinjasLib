package frc.lib.NinjasLib.swerve.constants;

public class SwerveModuleConstants {
    public int moduleNumber;
    public int driveMotorID;
    public int steerMotorID;
    public boolean driveMotorInverted;
    public boolean steerMotorInverted;
    public int canCoderID;
    public boolean invertCANCoder;
    public double CANCoderOffset;

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
