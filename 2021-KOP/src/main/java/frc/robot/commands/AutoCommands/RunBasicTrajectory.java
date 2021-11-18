// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunBasicTrajectory extends CommandBase {
  private Pose2d currentPosition;
  private DrivetrainSubsystem m_drivetrain;
  private Trajectory target;
  private Trajectory.State state = new Trajectory.State();
  private TrajectoryConfig trajConfig;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private ProfiledPIDController rot_pid;
  private Pose2d initialPos;
  HolonomicDriveController hController;

  private final Timer timer = new Timer();

  public RunBasicTrajectory(DrivetrainSubsystem m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    trajConfig = Constants.auto.follower.T_CONFIG.setKinematics(m_drivetrain.getKinematics());
    target = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1,0)),
        new Pose2d(2, 0, new Rotation2d(0)), trajConfig);
    timer.start();
  }

  @Override
  public void initialize() {

    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.auto.follower.X_PID_CONTROLLER,
        Constants.auto.follower.Y_PID_CONTROLLER, Constants.auto.follower.ROT_PID_CONTROLLER);
    hController.setEnabled(true);
  }

  @Override
  public void execute() {

    state = target.sample(timer.get());
    SmartDashboard.putNumber("Auto-Simple Timer", timer.get());
    currentPosition = m_drivetrain.getPose2d();
    speeds = hController.calculate(currentPosition, state, initialPos.getRotation());
    m_drivetrain.setAllStates(m_drivetrain.getKinematics().toSwerveModuleStates(speeds));
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(target.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.defense();
  }

}
