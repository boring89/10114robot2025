// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final LimeLightSubsystem limelightSubsystem = new LimeLightSubsystem();

  private final Joystick m_Joystick = new Joystick(OIConstants.kDriverControllerPort);
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> -m_Joystick.getRawAxis(OIConstants.kDriverYAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverXAxis), 
      () -> m_Joystick.getRawAxis(OIConstants.kDriverRotAxis), 
      () -> !m_Joystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
      
      new LimelightCmd(limelightSubsystem, swerveSubsystem);
    new JoystickButton(m_Joystick, 3).whileTrue(
      new SwerveJoystickCmd(
        swerveSubsystem, 
        () -> -(new LimeLightSubsystem().getYMove() * 0.3), 
        () -> new LimeLightSubsystem().getXMove() * 0.3, 
        () -> new LimeLightSubsystem().getYaw() * 0.3, 
        () -> false)
    );

    NamedCommands.registerCommand("Shoot", coralSubsystem.Shoot());
    NamedCommands.registerCommand("Intake", coralSubsystem.Intake());

    new InstantCommand();
    configureButtonBindings();
  }


  private void configureButtonBindings() {
    new JoystickButton(m_Joystick, 2).whileTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(m_Joystick, 5).whileTrue(new InstantCommand(() -> algaeSubsystem.ChangeMode()));
    new JoystickButton(m_Joystick, 6).whileTrue(new InstantCommand(() -> coralSubsystem.ChangeLevel()));
    new JoystickButton(m_Joystick, 1).whileTrue(new InstantCommand(() -> coralSubsystem.ChangeIntakeMode()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public PathPlannerAuto getAutonomousCommand(){
    return new PathPlannerAuto("New Auto");
  }
}
