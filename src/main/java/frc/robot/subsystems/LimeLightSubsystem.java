package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {

    private NetworkTable table, table2;
    public NetworkTableEntry tx1, tx2, ty1;
    private PIDController MoveX;
    
    public LimeLightSubsystem() {

        table = NetworkTableInstance.getDefault().getTable("limelight-up");
        table2 = NetworkTableInstance.getDefault().getTable("limelight-down");

        tx1 = table.getEntry("tx");

        tx2 = table2.getEntry("tx");

        MoveX = new PIDController(0.03, 0, 0);
    }

    public double getStationX() {
        return MoveX.calculate(tx1.getDouble(0.0), 0); 
    }

    public double getReefX(double mode) {
        return MoveX.calculate(tx2.getDouble(0.0), mode);
    }

    @Override
    public void periodic() {

    }
}