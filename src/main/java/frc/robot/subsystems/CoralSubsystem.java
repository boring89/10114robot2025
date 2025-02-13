package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    
    private final SparkMax ElevatorMotor, IntakeMotor;

    private final RelativeEncoder ElevatorEncoder;

    private final PIDController ElevatorPID;

    private int Level = 0;
    private int IntakeMode = 0;
    private boolean isIntake = false;

    public CoralSubsystem() {

        ElevatorMotor = new SparkMax(15, MotorType.kBrushless);
        IntakeMotor = new SparkMax(16, MotorType.kBrushless);

        ElevatorEncoder = ElevatorMotor.getEncoder();

        ElevatorPID = new PIDController(
            CoralConstants.kP, 
            CoralConstants.kI, 
            CoralConstants.kD);
    }

    public double getElevatorPosition() {
        return ElevatorEncoder.getPosition();
    }

    public void ChangeLevel() { 
        switch (Level) {
            case 0: case 1: case 2:
                Level += 1;
                break;
            case 3:
                Level = 0;
                break;
        }
    }

    public void ChangeIntakeMode() {
        switch (IntakeMode) {
            case 0:
                IntakeMode = 1;
                break;
            case 1:
                IntakeMode += 1;
                break;
            case 2:
                IntakeMode += 1;
                break;
            case 3:
                IntakeMode = 0;
                break;
        }
    }

    public void setElevatorPosition(double position) {
        ElevatorMotor.set(ElevatorPID.calculate(getElevatorPosition(), position));
    }

    public void setLevel() {
        switch (Level) {
            case 0:
                setElevatorPosition(CoralConstants.kCoralStation);
                break;
            case 1:
                setElevatorPosition(CoralConstants.kLevel1);
                break;
            case 2:
                setElevatorPosition(CoralConstants.kLevel2);
                break;
            case 3:
                setElevatorPosition(CoralConstants.kLevel3);
                break;
        }
    }

    public void Wait() {
        IntakeMotor.set(0);
    }

    public void Intake() {
        Level = 0;
        IntakeMotor.set(-0.2);
        isIntake = true;
    }

    public void Shoot() {
        IntakeMotor.set(0.5);
        isIntake = false;
    }

    public void initialize() {
        ElevatorEncoder.setPosition(0);
        System.out.println("CoralSubsystem initialized");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator", Level);
        SmartDashboard.putBoolean("isIntake", isIntake);

        if (IntakeMode == 0) {
            Wait();
        }else if (IntakeMode == 1) {
            Intake();
        }else if (IntakeMode == 2) {
            Wait();
        }else if (IntakeMode == 3) {
            Shoot();
        }
        setLevel();
    }

}
