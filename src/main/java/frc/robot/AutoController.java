package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.*;
import frc.robot.subsystems.Swerve;

public class AutoController {
    private Swerve m_swerve;
    SendableChooser<AutoSelection> autoSelector;
    AutoSelection selectedAuto;

    public AutoController(Swerve m_swerveIn) {
        m_swerve = m_swerveIn;
        
        autoSelector = new SendableChooser<AutoSelection>();

        // Add each auto to the dashboard dropdown menu
        for (AutoSelection auto : AutoSelection.values()) {
            autoSelector.addOption(auto.name(), auto);
        }

        SmartDashboard.putData("Auto Selector", autoSelector);
    }

    public AutoMode[] autos = {
        new ExampleAuto(m_swerve)
    };

    public enum AutoSelection {
        EXAMPLE_AUTO,
        EXAMPLE_AUTO_2
    };

    public void initialiseAuto() {
        selectedAuto = autoSelector.getSelected();
        SmartDashboard.putString("Sel auto", "" + selectedAuto);
    }

    public void runAuto() {
        SmartDashboard.putString("Sel auto", "" + selectedAuto);
    }
}
