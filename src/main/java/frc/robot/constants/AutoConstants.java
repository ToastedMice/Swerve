package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;  
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                6.28, 3.14);
}
