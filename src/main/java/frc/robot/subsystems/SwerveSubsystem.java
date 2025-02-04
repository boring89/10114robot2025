package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final CANcoder FLcancoder = new CANcoder(9, "rio");
    private final CANcoder FRcancoder = new CANcoder(10, "rio");
    private final CANcoder BLcancoder = new CANcoder(11, "rio");
    private final CANcoder BRcancoder = new CANcoder(12, "rio");
    
    private final SwerveModule FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort, 
        DriveConstants.kFLTurningMotorPort, 
        DriveConstants.kFLDriveEncoderReversed, 
        DriveConstants.kFLTurningEncoderReversed, 
        DriveConstants.kFLDriveAbsoluteEncoderPort, 
        DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFLDriveAbsoluteEncoderReversed);
    
    private final SwerveModule FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort, 
        DriveConstants.kFRTurningMotorPort, 
        DriveConstants.kFRDriveEncoderReversed, 
        DriveConstants.kFRTurningEncoderReversed, 
        DriveConstants.kFRDriveAbsoluteEncoderPort, 
        DriveConstants.kFRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFRDriveAbsoluteEncoderReversed);

    private final SwerveModule BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort, 
        DriveConstants.kBLTurningMotorPort, 
        DriveConstants.kBLDriveEncoderReversed, 
        DriveConstants.kBLTurningEncoderReversed, 
        DriveConstants.kBLDriveAbsoluteEncoderPort, 
        DriveConstants.kBLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBLDriveAbsoluteEncoderReversed);
    
    private final SwerveModule BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort, 
        DriveConstants.kBRTurningMotorPort, 
        DriveConstants.kBRDriveEncoderReversed, 
        DriveConstants.kBRTurningEncoderReversed, 
        DriveConstants.kBRDriveAbsoluteEncoderPort, 
        DriveConstants.kBRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBRDriveAbsoluteEncoderReversed);

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
        } catch (Exception e) {
        }
        }).start();;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        FL.stop();
        FR.stop();
        BL.stop();
        BR.stop();
    }

    public void resetEncoder() {
        FL.resetEncoders();
        FR.resetEncoders();
        BL.resetEncoders();
        BR.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        FL.setDesiredState(desiredStates[0]);
        FR.setDesiredState(desiredStates[1]);
        BL.setDesiredState(desiredStates[2]);
        BR.setDesiredState(desiredStates[3]);
    }

    public void getCANcoderPosition() {
        SmartDashboard.putNumber("FL", FLcancoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
        SmartDashboard.putNumber("FR", FRcancoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
        SmartDashboard.putNumber("BL", BLcancoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
        SmartDashboard.putNumber("BR", BRcancoder.getAbsolutePosition().getValueAsDouble() * (Math.PI * 2));
    }
}
