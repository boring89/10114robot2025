package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightCmd extends Command {
    private final LimeLightSubsystem limelightSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    public double TurnOutput, XMoveOutput, YMoveOutput;

    public final double getXMove() {
        return XMoveOutput;
    }
    public final double getYaw() {
        return TurnOutput;
    }
    public final double getYMove() {
        return YMoveOutput;
    }
    
    public LimelightCmd(LimeLightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(limelightSubsystem, swerveSubsystem);
        execute();
    }

    @Override
    public void execute() {
        TurnOutput = limelightSubsystem.getYaw();
        XMoveOutput = limelightSubsystem.getXMove();
        YMoveOutput = limelightSubsystem.getYMove();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
