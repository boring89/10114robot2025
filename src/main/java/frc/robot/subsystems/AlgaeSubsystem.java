package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax ArmMotor, IntakeMotor;

    private final RelativeEncoder ArmEncoder;

    private final PIDController ArmPID, CatchPID;

    private double IntakeMode = 0;

    public AlgaeSubsystem() {
        ArmMotor = new SparkMax(13, MotorType.kBrushless);
        IntakeMotor = new SparkMax(14, MotorType.kBrushless);
        ArmEncoder = ArmMotor.getEncoder();
        ArmPID = new PIDController(
            AlgaeConstants.kP, 
            AlgaeConstants.kI, 
            AlgaeConstants.kD);
        CatchPID = new PIDController(
            AlgaeConstants.kPcatch, 
            AlgaeConstants.kI, 
            AlgaeConstants.kD);
    }

    public double getArmPosition() {
        return ArmEncoder.getPosition();
    }

    public void ChangeMode() {
        if (IntakeMode < 3) {
            IntakeMode++;
        } else if (IntakeMode == 3) {
            IntakeMode = 0;
        }
    }

    public void Wait() {
        ArmMotor.set(CatchPID.calculate(getArmPosition(), 0));
        IntakeMotor.set(0);
    }

    public void catchAlgae() {
        ArmMotor.set(ArmPID.calculate(getArmPosition(), 16));
        IntakeMotor.set(-0.2);
    }

    public void goBack() {
        ArmMotor.set(CatchPID.calculate(getArmPosition(), 10));
        IntakeMotor.set(-0.1);
    }

    public void Shoot() {
        IntakeMotor.set(0.5);
    }

    public void initialize() {
        ArmEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        if (IntakeMode == 0) {
            Wait();
        }else if (IntakeMode == 1) {
            catchAlgae();
        }else if (IntakeMode == 2) {
            goBack();
        }else if (IntakeMode == 3) {
            Shoot();
        }
    }
}
