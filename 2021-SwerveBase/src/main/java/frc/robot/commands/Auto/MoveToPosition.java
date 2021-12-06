// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToPosition extends CommandBase {
  DriveSubsystem m_DrivetrainSubsystem;
  Pose2d desiredPosition;
  /** Creates a new MoveToPosition. */
  public MoveToPosition(DriveSubsystem m_DrivetrainSubsystem, Pose2d desiredPosition) {
    
    this.m_DrivetrainSubsystem = m_DrivetrainSubsystem;
    this.desiredPosition = desiredPosition;
    addRequirements(m_DrivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  
  /** 
   * @param execute(
   */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivetrainSubsystem.trajectoryFollow(desiredPosition);
  }

  
  /** 
   * @param interrupted
   */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_DrivetrainSubsystem.defense();
  }

  
  /** 
   * @return boolean
   */
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_DrivetrainSubsystem.finishedMovement();
  }

}
