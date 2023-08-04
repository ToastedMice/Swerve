package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class ExampleAuto {
    public ExampleAuto(Swerve m_swerve) {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(SwerveConstants.swerveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // start of the origin facing in the X+ direction
            new Pose2d(0, 0, new Rotation2d(0)), 

            // pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1), 
                new Translation2d(2, -1)
            ), 

            // end 3 meters straight ahead of where we started, facing forward 
            new Pose2d(3, 0, new Rotation2d(0)), 
        config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 
            0, 
            0, 
            new TrapezoidProfile.Constraints(6.28, 3.14));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0), thetaController);
        //Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp());
        Trajectory.State goal = trajectory.sample(3.4);
        
        ChassisSpeeds chassisSpeeds = controller.calculate(m_swerve.getPose(), goal, Rotation2d.fromDegrees(70.0));
        
        m_swerve.resetOdometry(trajectory.getInitialPose());

        SwerveModuleState[] moduleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setSwerveModuleStates(moduleStates);

    }
}
