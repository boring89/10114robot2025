package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
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
    
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, 
        gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            FL.getPosition(), 
            FR.getPosition(), 
            BL.getPosition(), 
            BR.getPosition()},
            new Pose2d(5.0, 13.5, new Rotation2d()));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
        } catch (Exception e) {
        }
        }).start();;
        setupPathPlanner();
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
        SmartDashboard.putNumber("FL", FL.getDriveVelocity());
        SmartDashboard.putNumber("FR", FR.getDriveVelocity());
        SmartDashboard.putNumber("BL", BL.getDriveVelocity());
        SmartDashboard.putNumber("BR", BR.getDriveVelocity());
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
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = FL.getPosition();
        positions[1] = FR.getPosition();
        positions[2] = BL.getPosition();
        positions[3] = BR.getPosition();
        return positions;
    }

    public void setupPathPlanner() {
        RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            FL.getPosition(), 
            FR.getPosition(), 
            BL.getPosition(), 
            BR.getPosition()};
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(this.getSwerveModuleState());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        setModuleStates(desiredStates);
    }

    public SwerveModuleState[] getSwerveModuleState() {
        return new SwerveModuleState[] {
            FL.getState(), 
            FR.getState(), 
            BL.getState(), 
            BR.getState()};
    }
    
}
