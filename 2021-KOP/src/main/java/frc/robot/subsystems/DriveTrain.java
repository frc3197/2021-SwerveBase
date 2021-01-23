// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  WPI_TalonFX motor1, motor2;

  SpeedControllerGroup leftSide = new SpeedControllerGroup(motor2);
  SpeedControllerGroup rightSide = new SpeedControllerGroup(motor1);
  
  DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);

  /** Creates a new DriveTrain. */
  public DriveTrain(int id1, int id2) {
    motor1 = new WPI_TalonFX(id1);
    motor2 = new WPI_TalonFX(id2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double l,double r){
    drive.tankDrive(l, r);
  }

  public void setSpeedLeft(double speed){
    motor2.set(speed);
  }

  public void setSpeedRight(double speed){
    motor1.set(speed);
  }


  public double getEncoderValLeft(){
    return motor2.getSelectedSensorPosition();
  }
  
  public double getEncoderValRight(){
    return motor1.getSelectedSensorPosition();
  }

}
