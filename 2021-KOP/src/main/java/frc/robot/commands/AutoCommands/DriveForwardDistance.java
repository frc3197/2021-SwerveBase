// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardDistance {
  /** Creates a new DriveForwardDistance. */
  Pose2d desiredPose2d;
  Pose2d currentPosition;
  double linearVelocity, newX, newY;
  DrivetrainSubsystem m_drivetrain;
  Trajectory target;
  TrajectoryConfig trajConfig;

  public DriveForwardDistance(DrivetrainSubsystem m_drivetrain, double distance) {
    this.m_drivetrain = m_drivetrain;
    currentPosition = m_drivetrain.getPose2d();
    newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVelocity = Constants.auto.follower.LINEAR_VELOCITY_DEFAULT;
    TrajectoryConfig trajConfig = Constants.auto.follower.T_CONFIG.setKinematics(m_drivetrain.getKinematics());
    target = TrajectoryGenerator.generateTrajectory(currentPosition, List.of(new Translation2d(newX * .25, newY * .25),
    new Translation2d(newX * .5, newY * .5), new Translation2d(newX * .75, newY * .75)),
        desiredPose2d, trajConfig);

  }

  public DriveForwardDistance(DrivetrainSubsystem m_drivetrain, double distance, double linearVelocity) {
    this.m_drivetrain = m_drivetrain;
    currentPosition = m_drivetrain.getPose2d();
    newX = currentPosition.getX() + (distance * currentPosition.getRotation().getCos());
    newY = currentPosition.getY() + (distance * currentPosition.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, currentPosition.getRotation());
    this.linearVelocity = linearVelocity;
    trajConfig = Constants.auto.follower.T_CONFIG.setKinematics(m_drivetrain.getKinematics());
    target = TrajectoryGenerator.generateTrajectory(currentPosition, List.of(new Translation2d(newX * .25, newY * .25),
        new Translation2d(newX * .5, newY * .5), new Translation2d(newX * .75, newY * .75)), desiredPose2d, trajConfig);

  }

  public Command getAutoCommand() {
    var rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand m_swerveCommand = new SwerveControllerCommand(target, m_drivetrain::getPose2d,
        m_drivetrain.getKinematics(), Constants.auto.follower.X_PID_CONTROLLER,
        Constants.auto.follower.Y_PID_CONTROLLER,rot_pid,
        m_drivetrain::setAllStates, m_drivetrain);
        m_drivetrain.resetOdometry(currentPosition);
        return m_swerveCommand.andThen( ()-> m_drivetrain.defense());
  }
}
