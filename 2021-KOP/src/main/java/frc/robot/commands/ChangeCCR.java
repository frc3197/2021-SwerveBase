// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChangeCCR extends InstantCommand {
  int index;

  /** Creates a new ChangeCCR. */
  public ChangeCCR(int index) {
    this.index = index;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double y = Constants.dimensions.TRACKWIDTH / 2.0;
    double x = Constants.dimensions.WHEELBASE / 2.0;
    switch(index){
      // The number represents the D-PAD in numpad notation
      // Eg. 1 represents bottom left
      case 1: checkCCR(new Translation2d(-y,x));
      case 2: checkCCR(new Translation2d(-y,0));        
      case 3: checkCCR(new Translation2d(-y,-x));
      case 4: checkCCR(new Translation2d(0,x));
      case 5: checkCCR(new Translation2d(0,0));
      case 6: checkCCR(new Translation2d(0,-x));
      case 7: checkCCR(new Translation2d(y,x));
      case 8: checkCCR(new Translation2d(y,0));
      case 9 : checkCCR(new Translation2d(y,-x));
    }

  }

  private void checkCCR(Translation2d newCCR){
    if(DrivetrainSubsystem.m_CCR == newCCR){
      DrivetrainSubsystem.m_CCR = new Translation2d(0,0);
    }
    else{
      DrivetrainSubsystem.m_CCR = newCCR;
    }
  }
}
