package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveConstants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends SubsystemBase {
    
    public SwerveModule[] swerveModules;
    public SwerveDriveOdometry odometry;

    private Field2d field;

    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };
        
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, null, null);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleNumber]);
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute () {
        for (SwerveModule module : swerveModules) {
            module.resetToAbsolute();
        }
    }
}

