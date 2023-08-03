package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase{
    private Swerve m_Swerve;
    private DoubleSupplier translationSupplier;
    private DoubleSupplier rotationSupplier;
    private DoubleSupplier strafeSupplier;

    public TeleopSwerve (Swerve swerve, DoubleSupplier translationSupplier, DoubleSupplier rotationSupplier, DoubleSupplier strafeSupplier) {
        m_Swerve = swerve;
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.strafeSupplier = strafeSupplier;
        addRequirements(m_Swerve);
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.stickDeadband);
        double rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.stickDeadband);
        double strafeValue = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.stickDeadband);
    
    
        m_Swerve.drive(
            new Translation2d(
                translationValue, strafeValue).times(SwerveConstants.maxSpeed), 
                rotationValue * SwerveConstants.maxAngularVelocity, 
                true
            );
    
    }
    
}
