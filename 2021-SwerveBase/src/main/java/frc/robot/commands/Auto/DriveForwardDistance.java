// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardDistance extends CommandBase{
  /** Creates a new DriveForwardDistance. */
  private Pose2d desiredPose2d;
  private Pose2d currentPosition;
  private double newX, newY;
  private DriveSubsystem m_drivetrain;
  private Trajectory target;
  private Trajectory.State state = new Trajectory.State();
  private TrajectoryConfig trajConfig;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private ProfiledPIDController rot_pid;
  private Pose2d initialPos;
  HolonomicDriveController hController;

  private final Timer timer = new Timer();

  public DriveForwardDistance(DriveSubsystem m_drivetrain, double distance) {
    this.m_drivetrain = m_drivetrain;
    initialPos = m_drivetrain.getPose2d();
    newX = initialPos.getX() + (distance * initialPos.getRotation().getCos());
    newY = initialPos.getY() + (distance * initialPos.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, initialPos.getRotation());
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    trajConfig = Constants.auto.follower.T_CONFIG.setKinematics(m_drivetrain.getKinematics());
    target = TrajectoryGenerator.generateTrajectory(initialPos, List.of(new Translation2d(newX * .25, newY * .25),
        new Translation2d(newX * .5, newY * .5), new Translation2d(newX * .75, newY * .75)), desiredPose2d, trajConfig);
    timer.start();
  }

  public DriveForwardDistance(DriveSubsystem m_drivetrain, double distance, double linearVelocity) {
    this.m_drivetrain = m_drivetrain;
    initialPos = m_drivetrain.getPose2d();
    newX = initialPos.getX() + (distance * initialPos.getRotation().getCos());
    newY = initialPos.getY() + (distance * initialPos.getRotation().getSin());
    this.desiredPose2d = new Pose2d(newX, newY, initialPos.getRotation());
    rot_pid = Constants.auto.follower.ROT_PID_CONTROLLER;
    trajConfig = Constants.auto.follower.T_CONFIG.setKinematics(m_drivetrain.getKinematics());
    target = TrajectoryGenerator.generateTrajectory(initialPos, List.of(new Translation2d(newX * .25, newY * .25),
    new Translation2d(newX * .5, newY * .5), new Translation2d(newX * .75, newY * .75)),
        desiredPose2d, trajConfig);
    
    timer.start();
  }
  @Override
  public void initialize() {
    
    rot_pid.enableContinuousInput(-Math.PI, Math.PI);
    hController = new HolonomicDriveController(Constants.auto.follower.X_PID_CONTROLLER, Constants.auto.follower.Y_PID_CONTROLLER, Constants.auto.follower.ROT_PID_CONTROLLER);
    hController.setEnabled(true);
  }

  @Override
  public void execute() {

    state = target.sample(timer.get());
    SmartDashboard.putNumber("Auto Timer", timer.get());
    currentPosition = m_drivetrain.getPose2d();
    speeds = hController.calculate(currentPosition, state, initialPos.getRotation());
    m_drivetrain.setAllStates(m_drivetrain.getKinematics().toSwerveModuleStates(speeds));
}


/** 
 * @return boolean
 */
@Override
public boolean isFinished() {
  return timer.hasElapsed(target.getTotalTimeSeconds());
}


/** 
 * @param interrupted
 */
@Override
public void end(boolean interrupted) {
    m_drivetrain.defense();
}


}
