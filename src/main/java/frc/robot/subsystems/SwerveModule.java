// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule{

  private final SparkFlex driveMotor;
  private final SparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  public static final SparkFlexConfig driveCfg = new SparkFlexConfig();
  public static final SparkMaxConfig turningCfg = new SparkMaxConfig();

  private final PIDController turningPidController;

  private final CANcoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;


  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    absoluteEncoder = new CANcoder(absoluteEncoderId);


  
    driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
  
    driveCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .inverted(driveMotorReversed);
    driveCfg.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    turningCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .inverted(turningMotorReversed);
    turningCfg.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
  
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
  
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  
    resetEncoders();
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
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }


  
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
  
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  
  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    
    
    state = SwerveModuleState.optimize(state, new Rotation2d(turningEncoder.getPosition()));
    
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
