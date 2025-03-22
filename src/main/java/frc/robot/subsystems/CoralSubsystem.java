package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

    private final SparkMax IntakeMotor;

    private final TalonFX ElevatorMotor;

    private final PIDController ElevatorPID;

    private int Level = 1;
    private int IntakeMode = 0;
    private double IntakeSpeed;
    private boolean isIntake = false;
    private int LastMode = 0;

    public CoralSubsystem() {

        ElevatorMotor = new TalonFX(15);
        IntakeMotor = new SparkMax(16, MotorType.kBrushless);

        ElevatorPID = new PIDController(
            CoralConstants.kP, 
            CoralConstants.kI, 
            CoralConstants.kD);
    }

    public double getElevatorPosition() {
        return ElevatorMotor.getPosition().getValueAsDouble();
    }

    public void ChangeLevel() { 
        switch (Level) {
            case 1: case 2:
                Level += 1;
                break;
            case 3:
                Level = 1;
                break;
        }
    }
    
    public void AutoLevel(int level) {
        Level = level;
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
                IntakeSpeed = 0.3;
                break;
            case 1:
                setElevatorPosition(CoralConstants.kLevel1);
                IntakeSpeed = -0.45;
                LastMode = 1;
                break;
            case 2:
                setElevatorPosition(CoralConstants.kLevel2);
                IntakeSpeed = -0.65;
                LastMode = 2;
                break;
            case 3:
                setElevatorPosition(CoralConstants.kLevel3);
                IntakeSpeed = -0.65;
                LastMode = 3;
                break;
        }
    }

    public void Wait() {
        if (isIntake == true) {
            Level = LastMode;
            isIntake = false;
        }
        IntakeMotor.set(0);
    }

    public void Intake() {
        Level = 0;
        IntakeMotor.set(IntakeSpeed);
        isIntake = true;
    }

    public void Shoot() {
        IntakeMotor.set(IntakeSpeed);
        System.out.println("hi");
        isIntake = false;
    }

    public void initialize() {
        ElevatorMotor.setPosition(0);
        System.out.println("CoralSubsystem initialized");
        UsbCamera CAM = CameraServer.startAutomaticCapture();
        CAM.setResolution(640, 480);
        CAM.setFPS(30);
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
