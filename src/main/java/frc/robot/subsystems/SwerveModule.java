package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.SwerveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.lib.math.Conversions;

public class SwerveModule {
    public int moduleNumber; 
    Rotation2d offset;

    TalonFX driveMotor;
    TalonFX steerMotor;
    CANCoder steerEncoder;

    SimpleMotorFeedforward simpleMotorFeedforward;
    

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        steerMotor = new TalonFX(moduleConstants.steerMotorID);
        steerEncoder = new CANCoder(moduleConstants.steerEncoderID);

        simpleMotorFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

        steerEncoder = new CANCoder(moduleConstants.steerEncoderID);
        
    }

    public void setDesiredState(SwerveModuleState desiredState) {

    }

    private void setSpeed(SwerveModuleState desiredState) {
        double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(steerEncoder.getAbsolutePosition(), SwerveConstants.steerGearRatio);
        driveMotor.setSelectedSensorPosition(absolutePosition);
    }

    

}
