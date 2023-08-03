package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int steerMotorID;
    public final int steerEncoderID;
    public Rotation2d offset = new Rotation2d(0);

    public SwerveModuleConstants (int driveMotorID, int steerMotorID, int steerEncoderID, Rotation2d offset) {
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.steerEncoderID = steerEncoderID;
        this.offset = offset;
    }
    
}
