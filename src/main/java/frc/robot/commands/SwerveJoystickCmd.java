// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter , turningLimiter;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSec);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSec);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    //get real-time input
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turningSpd = turningSpdFunction.get();

    //deadband
    xSpd = Math.abs(xSpd) > OIConstants.kDeadBand ? xSpd : 0.0;
    ySpd = Math.abs(ySpd) > OIConstants.kDeadBand ? ySpd : 0.0;
    turningSpd = Math.abs(turningSpd) >OIConstants.kDeadBand ? turningSpd : 0.0;

    //smoother-driving
    xSpd = xLimiter.calculate(xSpd) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
    ySpd = yLimiter.calculate(ySpd) * DriveConstants.kTeleDriveMaxSpeedMeterPerSec;
    turningSpd = turningLimiter.calculate(turningSpd)
      * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSec;

    //chassis speed
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      //relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpd, ySpd, turningSpd, swerveSubsystem.getRotation2d()
      );
    } else {
      //relative to robot's head
      chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turningSpd);
    }

    // convert chassis speed to module state
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    //Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }


  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
