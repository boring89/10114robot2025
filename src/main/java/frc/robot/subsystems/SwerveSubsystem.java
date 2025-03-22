package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveModule FL, FR, BL, BR;
    private final AHRS gyro;
    
    public SwerveSubsystem() {
        FL = new SwerveModule(
        DriveConstants.kFLDriveMotorPort, 
        DriveConstants.kFLTurningMotorPort, 
        DriveConstants.kFLDriveEncoderReversed, 
        DriveConstants.kFLTurningEncoderReversed, 
        DriveConstants.kFLDriveAbsoluteEncoderPort, 
        DriveConstants.kFLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFLDriveAbsoluteEncoderReversed);
    
        FR = new SwerveModule(
        DriveConstants.kFRDriveMotorPort, 
        DriveConstants.kFRTurningMotorPort, 
        DriveConstants.kFRDriveEncoderReversed, 
        DriveConstants.kFRTurningEncoderReversed, 
        DriveConstants.kFRDriveAbsoluteEncoderPort, 
        DriveConstants.kFRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFRDriveAbsoluteEncoderReversed);

        BL = new SwerveModule(
        DriveConstants.kBLDriveMotorPort, 
        DriveConstants.kBLTurningMotorPort, 
        DriveConstants.kBLDriveEncoderReversed, 
        DriveConstants.kBLTurningEncoderReversed, 
        DriveConstants.kBLDriveAbsoluteEncoderPort, 
        DriveConstants.kBLDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBLDriveAbsoluteEncoderReversed);
    
        BR = new SwerveModule(
        DriveConstants.kBRDriveMotorPort, 
        DriveConstants.kBRTurningMotorPort, 
        DriveConstants.kBRDriveEncoderReversed, 
        DriveConstants.kBRTurningEncoderReversed, 
        DriveConstants.kBRDriveAbsoluteEncoderPort, 
        DriveConstants.kBRDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBRDriveAbsoluteEncoderReversed);

        gyro = new AHRS(NavXComType.kMXP_SPI);

        resetEncoder();
        zeroHeading();

        odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                FL.getPosition(),
                FR.getPosition(),
                BL.getPosition(),
                BR.getPosition()
      });

        setupPathPlanner();

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getRotation2d(),
            new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()},
            getPose());

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FL", FL.getDriveVelocity());
        SmartDashboard.putNumber("FR", FR.getDriveVelocity());
        SmartDashboard.putNumber("BL", BL.getDriveVelocity());
        SmartDashboard.putNumber("BR", BR.getDriveVelocity());
        m_poseEstimator.update(getRotation2d(),      
        new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()});

        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                FL.getPosition(),
                FR.getPosition(),
                BL.getPosition(),
                BR.getPosition()
                });
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

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            FL.getPosition(), 
            FR.getPosition(), 
            BL.getPosition(), 
            BR.getPosition()
        };
    }    

    public void setupPathPlanner() {
        RobotConfig config = null;
        try{
          config = RobotConfig.fromGUISettings();
        }catch(Exception e) {
          e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetPose, 
            this::getdriveRobotRelative, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                    new PIDConstants(2.8, 0.001, 0.006),  // 6,0,0
                    new PIDConstants(3.0, 0.005, 0.0)  // 5,0,0
            ),
            config,
            () -> {

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(
        Rotation2d.fromDegrees(0),//gyro.getAngle()),
        new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        },
        pose);
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            FL.getPosition(), 
            FR.getPosition(), 
            BL.getPosition(), 
            BR.getPosition()};
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            FL.getState(), 
            FR.getState(), 
            BL.getState(), 
            BR.getState()
        );    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)); 
    }

    public ChassisSpeeds getdriveRobotRelative() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
      FL.getState(),
      FR.getState(),
      BL.getState(),
      BR.getState());
      }
}
