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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class ExampleAuto implements AutoMode {
    private final Swerve m_swerve;
    private final HolonomicDriveController controller;
    private final Trajectory trajectory;
    Trajectory.State goal;
    public boolean isFinished = false;
    private double startTime;

    public ExampleAuto(Swerve m_swerveIn) {
        m_swerve = m_swerveIn;

        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        ).setKinematics(SwerveConstants.swerveKinematics);

        trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );

        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController,
            0,
            0,
            new TrapezoidProfile.Constraints(6.28, 3.14)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController
        );
        m_swerve.resetOdometry(this.trajectory.getInitialPose());
        startTime = Timer.getFPGATimestamp();
    }

    public void run() {
        goal = this.trajectory.sample(Timer.getFPGATimestamp()-startTime);
        ChassisSpeeds chassisSpeeds = this.controller.calculate(
            m_swerve.getPose(),
            goal,
            Rotation2d.fromDegrees(70.0)
        );

        if (goal == trajectory.getStates().get(trajectory.getStates().size()-1)) {
            isFinished = true;
        }

        SwerveModuleState[] moduleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setSwerveModuleStates(moduleStates);

        SmartDashboard.putNumber("Module1 Speed Auto", moduleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Module1 Angle Auto", moduleStates[0].angle.getDegrees());
    }

    public String getNameString() {
        return "Example Auto";
    }
}
