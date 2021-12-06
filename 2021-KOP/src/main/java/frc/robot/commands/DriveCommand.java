package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private boolean fieldRelative = Constants.swerve.feildRelativeOn;
    private boolean brakeMode = Constants.swerve.brakeModeOn;

    public DriveCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        brakeMode = DriveSubsystem.getBrakeMode();
        fieldRelative = DriveSubsystem.getFieldRelative();

        if(!brakeMode){
        m_drivetrainSubsystem.drive(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble(),
                m_drivetrainSubsystem.getGyroscopeRotation()) : new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble()));
        }
        else{
        if (m_translationXSupplier.getAsDouble() != 0 && m_translationYSupplier.getAsDouble() != 0
                && m_rotationSupplier.getAsDouble() != 0) {
            m_drivetrainSubsystem.drive(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble(),
                m_drivetrainSubsystem.getGyroscopeRotation()) : new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(), m_rotationSupplier.getAsDouble()));
        } else {
            m_drivetrainSubsystem.defense();
        }}
    }

    
    /** 
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}