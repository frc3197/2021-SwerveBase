// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Toggles;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
// ONLY WORKS AS A TOGGLE COMMAND
public class Defend extends CommandBase {
  DriveSubsystem m_drivetrain;
  /** Creates a new Defend. */
  // Works as a psuedo manual brake by moving the swerve modules into an X configuration. This can help us not get bullied/pushed if neccesary or just as a quick stop.
  public Defend(DriveSubsystem m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    this.m_drivetrain = m_drivetrain;
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
    m_drivetrain.defense();
  }

  
  /** 
   * @param isFinished(
   */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  
  /** 
   * @return boolean
   */
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
