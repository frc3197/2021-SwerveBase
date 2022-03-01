package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;

    private double inputX;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.swerve.xLimVal);

    private final DoubleSupplier m_translationYSupplier;
    
    private double inputY;
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.swerve.yLimVal);

    private final DoubleSupplier m_rotationSupplier;

    private double inputRot;
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.swerve.rotLimVal);

    
    private String RobotDriveTrainState = "";

    private boolean fieldRelative;
    private boolean brakeMode;

    public DriveCommand(DriveSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        fieldRelative = DriveSubsystem.getFieldRelative();
        brakeMode = DriveSubsystem.getBrakeMode();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        brakeMode = DriveSubsystem.getBrakeMode();
        fieldRelative = DriveSubsystem.getFieldRelative();
        // Stored in a new object for Oblog functionality
        inputX = m_translationXSupplier.getAsDouble();
        inputY = m_translationYSupplier.getAsDouble();
        inputRot = m_rotationSupplier.getAsDouble();

        if (!brakeMode) {
            RobotDriveTrainState = "Brake Mode Off";
            m_drivetrainSubsystem.drive(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xLimiter.calculate(inputX), yLimiter.calculate(inputY),
                            rotLimiter.calculate(inputRot),
                            m_drivetrainSubsystem.getGyroscopeRotation())
                    : new ChassisSpeeds(xLimiter.calculate(inputX), yLimiter.calculate(inputY),
                            rotLimiter.calculate(inputRot)));
        } else {
            if (inputX != 0.0 || inputY != 0.0 || inputRot != 0.0) {
                RobotDriveTrainState = "Brake Mode On - Driving";
                m_drivetrainSubsystem.drive(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xLimiter.calculate(inputX), yLimiter.calculate(inputY),
                                rotLimiter.calculate(inputRot),
                                m_drivetrainSubsystem.getGyroscopeRotation())
                        : new ChassisSpeeds(xLimiter.calculate(inputX), yLimiter.calculate(inputY),
                                rotLimiter.calculate(inputRot)));
            } else {
                RobotDriveTrainState = "Brake Mode On - Standby";
                m_drivetrainSubsystem.defense();
            }
        }
    }

    /**
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
