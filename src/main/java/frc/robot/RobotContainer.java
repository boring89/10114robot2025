// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final LimeLightSubsystem limelightSubsystem = new LimeLightSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -m_Joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> true));
      
      new LimelightCmd(limelightSubsystem, swerveSubsystem);
    new JoystickButton(m_Joystick, 3).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> 0d, 
        () -> new LimeLightSubsystem().getStationX() * 0.3, 
        () -> 0d, 
        () -> false));

    new POVButton(m_Joystick, 270).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> 0d, 
        () -> new LimeLightSubsystem().getReefX(40) * 0.2, 
        () -> 0d, 
        () -> false));
    
    new POVButton(m_Joystick, 90).whileTrue(
      new SwerveJoystickCmd(  
        swerveSubsystem, 
        () -> 0d, 
        () -> new LimeLightSubsystem().getReefX(-8.7)* 0.2, 
        () -> 0d, 
        () -> false));
    

    NamedCommands.registerCommand("shoot", new RunCommand(() -> coralSubsystem.Shoot(), coralSubsystem).withTimeout(3));
    // NamedCommands.registerCommand("apriltag", new SwerveJoystickCmd(
    //                                                                       swerveSubsystem, 
    //                                                                       () -> -limelightSubsystem.getReefY(), 
    //                                                                       () -> limelightSubsystem.getReefX(20) * 0.2, 
    //                                                                       () -> limelightSubsystem.getReefYaw() * 0.3, 
    //                                                                       () -> false).withTimeout(5));
    NamedCommands.registerCommand("l3",  new RunCommand(() -> coralSubsystem.AutoLevel(3), coralSubsystem).withTimeout(4));


    new InstantCommand();
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    new JoystickButton(m_Joystick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(m_Joystick, 6).whileTrue(new InstantCommand(() -> coralSubsystem.ChangeLevel()));
    new JoystickButton(m_Joystick, 1).whileTrue(new InstantCommand(() -> coralSubsystem.ChangeIntakeMode()));
    new POVButton(m_Joystick, 0).whileTrue(new RunCommand(() -> armSubsystem.ArmUp()));
    new POVButton(m_Joystick, 180).whileTrue(new RunCommand(() -> armSubsystem.ArmDown()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    return new SwerveJoystickCmd(swerveSubsystem, () -> -0.4, () -> 0.0, () -> 0.0, () -> false).withTimeout(6)
    .andThen(new InstantCommand(() -> System.out.println("test"))).withTimeout(2)
    .andThen(new RunCommand(() -> coralSubsystem.Shoot(), coralSubsystem)).withTimeout(3);
  }
}

