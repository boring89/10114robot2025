// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 5.95;
    public static final double kTurningMotorGearRatio = 1 / 21.4;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.3;
  }

  public static final class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(21);    //distance between left and right wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);   //distance between front and rear wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final int kFLDriveMotorPort = 1;
      public static final int kFRDriveMotorPort = 2;
      public static final int kBLDriveMotorPort = 3;
      public static final int kBRDriveMotorPort = 4;
      
      public static final int kFLTurningMotorPort = 5;
      public static final int kFRTurningMotorPort = 6;
      public static final int kBLTurningMotorPort = 7;
      public static final int kBRTurningMotorPort = 8;

      public static final boolean kFLDriveEncoderReversed = false;
      public static final boolean kFRDriveEncoderReversed = true;
      public static final boolean kBLDriveEncoderReversed = false;
      public static final boolean kBRDriveEncoderReversed = true;

      public static final boolean kFLTurningEncoderReversed = false;
      public static final boolean kFRTurningEncoderReversed = false;
      public static final boolean kBLTurningEncoderReversed = false;
      public static final boolean kBRTurningEncoderReversed = false;

      public static final int kFLDriveAbsoluteEncoderPort = 9;
      public static final int kFRDriveAbsoluteEncoderPort = 10;
      public static final int kBLDriveAbsoluteEncoderPort = 11;
      public static final int kBRDriveAbsoluteEncoderPort = 12;

      public static final boolean kFLDriveAbsoluteEncoderReversed = false;
      public static final boolean kFRDriveAbsoluteEncoderReversed = false;
      public static final boolean kBLDriveAbsoluteEncoderReversed = false;
      public static final boolean kBRDriveAbsoluteEncoderReversed = true;

      public static final double kFLDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kFRDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kBLDriveAbsoluteEncoderOffsetRad = 0;
      public static final double kBRDriveAbsoluteEncoderOffsetRad = 0;

      public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
      public static final double kPhysicalMaxAngularSpeedRadiansPerSec = 3 * 1.6 * Math.PI;
      public static final double kTeleDriveMaxSpeedMeterPerSec = (kPhysicalMaxSpeedMetersPerSecond / 4);
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSec = //
          kPhysicalMaxAngularSpeedRadiansPerSec / 4;
      public static final double kTeleDriveMaxAccelerationUnitsPerSec = 3;
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSec = 3;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadBand = 0.06;
  }
}
