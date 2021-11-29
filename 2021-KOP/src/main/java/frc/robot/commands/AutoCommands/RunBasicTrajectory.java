// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.extra_libraries.PathPlanner;
import frc.robot.extra_libraries.PathPlannerTrajectory;
import frc.robot.extra_libraries.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RunBasicTrajectory extends CommandBase {
  private Pose2d currentPosition;
  private DrivetrainSubsystem m_drivetrain;
  private PathPlannerTrajectory target;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private ProfiledPIDController rot_pid;
  private PathPlannerState state;
  private HolonomicDriveController hController;

  private final Timer timer = new Timer();

  public RunBasicTrajectory(DrivetrainSubsystem m_drivetrain, String path) {
    this.m_drivetrain = m_drivetrain;
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    target = PathPlanner.loadPath(path, Constants.swerve.MAX_VEL_METERS, Constants.swerve.MAX_ANG_VEL_RAD);
  }

  @Override
  public void initialize() {

    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.auto.follower.X_PID_CONTROLLER,
        Constants.auto.follower.Y_PID_CONTROLLER, Constants.auto.follower.ROT_PID_CONTROLLER);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    var curTime = timer.get();
    state = (PathPlannerState) target.sample(curTime);
    currentPosition = m_drivetrain.getPose2d();
    speeds = hController.calculate(currentPosition, state, state.holonomicRotation);
    m_drivetrain.setAllStates(m_drivetrain.getKinematics().toSwerveModuleStates(speeds));
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(target.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_drivetrain.defense();
  }

}
