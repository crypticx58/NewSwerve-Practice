// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldPositioningConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.InputsManager.AutoPositioningInputsManager;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPositioningCommand extends Command {
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final AutoPositioningInputsManager autoPositioningInputsManager;
  private Pose2d currentTargetPose;
  private Command currentPathPlanner;
  /** Creates a new AutoPositioningCommand. */
  public AutoPositioningCommand(AutoPositioningInputsManager autoPositioningInputsManager) {
    this.autoPositioningInputsManager = autoPositioningInputsManager;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (autoPositioningInputsManager.getLeftSpeaker()){
        currentTargetPose = FieldPositioningConstants.getAllianceSpeakerLeftPose();
      } else if (autoPositioningInputsManager.getRightSpeaker()){
        currentTargetPose = FieldPositioningConstants.getAllianceSpeakerRightPose();
      }else if (autoPositioningInputsManager.getCenterSpeaker()){
        currentTargetPose = FieldPositioningConstants.getAllianceSpeakerMiddlePose();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
