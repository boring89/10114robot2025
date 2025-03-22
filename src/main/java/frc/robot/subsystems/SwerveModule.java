// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule{

  private final SparkFlex driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  public static final SparkFlexConfig driveCfg = new SparkFlexConfig();
  public static final SparkMaxConfig turningCfg = new SparkMaxConfig();

  private final SparkClosedLoopController drivingPidController;
  private final SparkClosedLoopController turningPidController;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;

  private double angle;

  private SwerveModuleState correctedDesiredState;

  double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    absoluteEncoder = new CANcoder(absoluteEncoderId);
  
    driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
  
    driveCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .inverted(driveMotorReversed)
        .apply(driveCfg);
    driveCfg.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
        .apply(driveCfg.encoder);
    driveCfg.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.13, 0.000, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
    turningCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(turningMotorReversed)
        .apply(turningCfg);
    turningCfg.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)
        .apply(turningCfg.encoder);
    turningCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(1.6, 0.0005, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);

    driveMotor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(turningCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
  
    drivingPidController = driveMotor.getClosedLoopController();
    turningPidController = turningMotor.getClosedLoopController();

  }
  
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }
  
  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }
  
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }
  
  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }
  
  public double getAbsoluteEncoderRad() {
    angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
    angle *= absoluteEncoderReversed ? -1.0d : 1.0d;
    SmartDashboard.putNumber("absolutEncoderAngle", angle);
    return angle;
  }


  
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),
     Rotation2d.fromRadians(getAbsoluteEncoderRad()));
  }
  
  public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = new SwerveModuleState();
    state.angle = Rotation2d.fromRadians(getTurningPosition());
    desiredState.optimize(state.angle);
    
    drivingPidController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    turningPidController.setReference(desiredState.angle.getRadians(),ControlType.kPosition);

    return correctedDesiredState;
  }

  public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
