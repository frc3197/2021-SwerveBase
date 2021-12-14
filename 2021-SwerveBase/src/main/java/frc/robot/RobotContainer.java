// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Auto.RunBasicTrajectory;
import frc.robot.commands.Toggles.Defend;
import frc.robot.commands.Toggles.ToggleBrakeMode;
import frc.robot.commands.Toggles.ToggleFieldRelative;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.FilteredController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static DriveSubsystem m_drivetrainSubsystem = new DriveSubsystem();

  private final static XboxController m_controller = new XboxController(0);
  public static final FilteredController filteredController = new FilteredController(m_controller);
  public static final DriveCommand m_driveCommand = new DriveCommand(
    m_drivetrainSubsystem,
    () -> -modifyAxis(filteredController.getY(GenericHID.Hand.kLeft,.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.outputs.strafe,
    () -> -modifyAxis(filteredController.getX(GenericHID.Hand.kLeft,.2)) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.outputs.strafe,
    () -> -modifyAxis(filteredController.getX(GenericHID.Hand.kRight,.2)) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.outputs.turnRate
);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
    
    recalibrateGyroscope();
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getYButton).whenPressed(m_driveCommand);
    //new Button(m_controller::getXButton).whenPressed(new MoveToPosition(m_drivetrainSubsystem, new Pose2d(0,0 , new Rotation2d(Math.PI))));
    new Button(m_controller::getXButton).whenPressed(new RunBasicTrajectory(m_drivetrainSubsystem, "Test Path Correct Ver"));
    new Button(m_controller::getAButton).whileHeld(new Defend(m_drivetrainSubsystem));
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(m_controller::getStartButton).whenPressed(new ToggleFieldRelative());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  public void resetOdometry(){
    m_drivetrainSubsystem.resetOdometry();
  }

  public void recalibrateGyroscope(){
    m_drivetrainSubsystem.getGyroscopeObj().calibrate();
    m_drivetrainSubsystem.getGyroscopeObj().reset();
    m_drivetrainSubsystem.getGyroscopeObj().zeroYaw();
  }

  
  /** 
   * @param value
   * @param deadband
   * @return double
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  
  /** 
   * @param value
   * @return double
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }



  public void publishPosition(){
    SmartDashboard.putNumber("CurrentPosX", m_drivetrainSubsystem.getPose2d().getX());
    SmartDashboard.putNumber("CurrentPosY", m_drivetrainSubsystem.getPose2d().getY());
    SmartDashboard.putNumber("CurrentPosRot", m_drivetrainSubsystem.getPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber("Controller POV", filteredController.getPOVButton());
    SmartDashboard.putBoolean("Controller POV Bool", filteredController.getPOVPressed());
    
  }
}
