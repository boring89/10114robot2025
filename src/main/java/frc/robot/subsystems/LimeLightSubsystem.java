package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

    private NetworkTable table;
    public NetworkTableEntry tx, ty, ledMode;
    private PIDController Turn, MoveX, MoveY;
    private double[] botPose;
    private boolean isApriltagDetected;
    
    public LimeLightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        Turn = new PIDController(0.08, 0, 0);
        MoveX = new PIDController(0.07, 0, 0);
        MoveY = new PIDController(0.04, 0, 0);
        botPose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    }

    public double getXMove() {
        return MoveX.calculate(tx.getDouble(0.0), 0); 
    }

    public double getYMove() {
        return MoveY.calculate(ty.getDouble(0.0), 24.5);
    }
    

    public double getYaw() {
        if (botPose.length >= 5) {
            return Turn.calculate(botPose[4], 0);
        }
        isApriltagDetected = false;
        return 0.0;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
        SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
        SmartDashboard.putNumber("LimmelightYaw", botPose[4]);
        SmartDashboard.putBoolean("Apriltag Detect", isApriltagDetected);
        
    }
}