// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardDistance extends CommandBase {
  /** Creates a new DriveForwardDistance. */
Pose2d desiredPose2d;
double linearVelocity;
DrivetrainSubsystem m_drivetrain;


  public DriveForwardDistance(DrivetrainSubsystem m_drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);

    Pose2d currentPosition = m_drivetrain.getPose2d();
    double newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    double newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVelocity = Constants.maximums.swerve.MAX_VEL_METERS;

  }

  public DriveForwardDistance(DrivetrainSubsystem m_drivetrain, double distance, double linearVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);

    Pose2d currentPosition = m_drivetrain.getPose2d();
    double newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    double newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVelocity = linearVelocity;

  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.trajectoryFollow(desiredPose2d, linearVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.finishedMovement();
  }
}
