package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve{
    private Swerve m_Swerve;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier rotationSupplier;
    private DoubleSupplier strafeSupplier;

    public TeleopSwerve (Swerve swerve, DoubleSupplier translationSupplier, DoubleSupplier rotationSupplier, DoubleSupplier strafeSupplier) {
        this.m_Swerve = swerve;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.strafeSupplier = strafeSupplier;
    }

    public void execute() {
        double translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.stickDeadzone);
        double rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.stickDeadzone);
        double strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.stickDeadzone);
    
    
        m_Swerve.drive(
            new Translation2d(translationValue, strafeValue).times(SwerveConstants.maxSpeed), 
                rotationValue * SwerveConstants.maxAngularVelocity, 
                true
            );
    
    }
    
}
