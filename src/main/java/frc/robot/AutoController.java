package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.*;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoController {
    private Swerve m_swerve;

    public AutoController(Swerve m_swerve) {
        this.m_swerve = m_swerve;
    }
    
    public enum AutoSelection {
        EXAMPLE_AUTO
    };

    // public AutoMode[] autos = {
    //     new ExampleAutoCopy(m_swerve)
    // };

    public Command autoCommand = new ExampleAutoCopy(m_swerve);
    //autoCommand.schedule();

    SendableChooser<AutoSelection> autoSelector = new SendableChooser<>();
    AutoSelection selectedAuto;

    static AutoMode runningAuto;

    private AutoController() {
		autoSelector = new SendableChooser<AutoSelection>();
		
		// Add each auto to the dashboard dropdown menu
		for (AutoSelection auto : AutoSelection.values()) {			
			autoSelector.addOption(auto.name(), auto);
		}

		SmartDashboard.putData("Auto Selector", autoSelector);
	}

    // public boolean exit() {
	// 	return runningAuto.getExit();
	// }

    
}
