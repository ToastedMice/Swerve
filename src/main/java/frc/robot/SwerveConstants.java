package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.SwerveModuleConstants;

public final class SwerveConstants {

    public static final double maxSpeed = 1;
    public static final double maxAngularVelocity = 1;

    public static final double trackWidth = 1; 
    public static final double wheelBase = 1; 

    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double steerGearRatio = 10.0;
    public static final double driveGearRatio = 10.0;

    public static final double wheelCircumference = 0.5;



    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


    public static final class Mod0 {
        public static final int driveMotorID = 0;
        public static final int steerMotorID = 0;
        public static final int steerEncoderID = 0;
        public static Rotation2d offset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steerMotorID, steerEncoderID, offset);
    }
    public static final class Mod1 {
        public static final int driveMotorID = 0;
        public static final int steerMotorID = 0;
        public static final int steerEncoderID = 0;
        public static Rotation2d offset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steerMotorID, steerEncoderID, offset);
    }
    public static final class Mod2 {
        public static final int driveMotorID = 0;
        public static final int steerMotorID = 0;
        public static final int steerEncoderID = 0;
        public static Rotation2d offset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steerMotorID, steerEncoderID, offset);
    }
    public static final class Mod3 {
        public static final int driveMotorID = 0;
        public static final int steerMotorID = 0;
        public static final int steerEncoderID = 0;
        public static Rotation2d offset = Rotation2d.fromDegrees(0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, steerMotorID, steerEncoderID, offset);
        }
}