package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;

public interface AutoMode {

    /**
     * Returns String of the name of the auto mode.
     * @return Name of auto mode.
     */
    public Command getAutoCommand();
}