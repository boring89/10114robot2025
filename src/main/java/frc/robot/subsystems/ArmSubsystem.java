package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    
    private final TalonFX ArmMotor;
    private final PIDController ArmController;
    private double position = 0;
    
    public ArmSubsystem() {
        ArmMotor = new TalonFX(17);
        ArmController = new PIDController(0.1, 0, 0);

    }

    public double getPosition() {
        return ArmMotor.getPosition().getValueAsDouble();
    }

    public void ArmUp() {
        if (getPosition() >= 50) {
            position+=0;
        } else {
            position++;
        }
    }

    public void ArmDown() {
        position--;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmPosition", getPosition());
        ArmMotor.set(ArmController.calculate(getPosition(), position));
    }
}
