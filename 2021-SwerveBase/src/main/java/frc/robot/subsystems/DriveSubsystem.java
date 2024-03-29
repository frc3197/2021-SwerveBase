// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
 */
        public static final double MAX_VOLTAGE = Constants.swerve.MAX_VOLTAGE;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
 */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.swerve.MAX_VEL_METERS;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
// replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.swerve.MAX_ANG_VEL_RAD;

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0),
                        // Front right
                        new Translation2d(Constants.dimensions.TRACKWIDTH / 2.0, -Constants.dimensions.WHEELBASE / 2.0),
                        // Back left
                        new Translation2d(-Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0),
                        // Back right
                        new Translation2d(-Constants.dimensions.TRACKWIDTH / 2.0,
                                        -Constants.dimensions.WHEELBASE / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        @Log
        private final AHRS m_navx = new AHRS(Port.kUSB); // NavX connected over MXP

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                        new Rotation2d(-getGyroscopeRotation().getDegrees()), Constants.auto.startingPos.DEFAULT_POS);

        private HolonomicDriveController follower = new HolonomicDriveController(
                        Constants.auto.follower.X_PID_CONTROLLER, Constants.auto.follower.Y_PID_CONTROLLER,
                        Constants.auto.follower.ROT_PID_CONTROLLER);

        public static Translation2d m_CCR = new Translation2d();
        @Log
        private static boolean brakeMode = Constants.swerve.brakeModeOn;
        @Log
        private static boolean fieldRelative = Constants.swerve.feildRelativeOn;
        
        public DriveSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                Constants.auto.follower.X_PID_CONTROLLER.setTolerance(.02);
                Constants.auto.follower.Y_PID_CONTROLLER.setTolerance(.02);
                Constants.auto.follower.ROT_PID_CONTROLLER.setTolerance(.02);
                follower.setTolerance(new Pose2d(.1, .1, new Rotation2d(Math.toRadians(5))));

                m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
                                                0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk3SwerveModuleHelper.GearRatio.FAST,
                                // This is the ID of the drive motor
                                Constants.modInfo.flMod.MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                Constants.modInfo.flMod.MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                Constants.modInfo.flMod.MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                Constants.modInfo.flMod.MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
                                                0),
                                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.modInfo.frMod.MODULE_DRIVE_MOTOR,
                                Constants.modInfo.frMod.MODULE_STEER_MOTOR,
                                Constants.modInfo.frMod.MODULE_STEER_ENCODER,
                                Constants.modInfo.frMod.MODULE_STEER_OFFSET);

                m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,
                                                0),
                                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.modInfo.blMod.MODULE_DRIVE_MOTOR,
                                Constants.modInfo.blMod.MODULE_STEER_MOTOR,
                                Constants.modInfo.blMod.MODULE_STEER_ENCODER,
                                Constants.modInfo.blMod.MODULE_STEER_OFFSET);

                m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
                                                0),
                                Mk3SwerveModuleHelper.GearRatio.FAST, Constants.modInfo.brMod.MODULE_DRIVE_MOTOR,
                                Constants.modInfo.brMod.MODULE_STEER_MOTOR,
                                Constants.modInfo.brMod.MODULE_STEER_ENCODER,
                                Constants.modInfo.brMod.MODULE_STEER_OFFSET);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        
        /** 
         * @return AHRS
         */
        public AHRS getGyroscopeObj() {
                return m_navx;
        }

        
        /** 
         * @return Rotation2d
         */
        public Rotation2d getGyroscopeRotation() {
                return m_navx.getRotation2d();
        }

        
        /** 
         * @param chassisSpeeds
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("New CCR X",m_CCR.getX());
                SmartDashboard.putNumber("New CCR Y",m_CCR.getY());
                SmartDashboard.putNumber("GyroOutputRaw", getGyroscopeRotation().getDegrees());
                SmartDashboard.putNumber("GyroOutputAuto", -getGyroscopeRotation().getDegrees()); // Left/CCW should
                                                                                                  // increase the gyro
                
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds, m_CCR);
                SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                updateOdometry(states);
                setAllStates(states);
               
                

        }

        
        /** 
         * @param states
         */
        public void updateOdometry(SwerveModuleState[] states) {
                m_odometry.update(Rotation2d.fromDegrees(-getGyroscopeRotation().getDegrees()), states[0], states[1],
                                states[2], states[3]);
        }

        
        /** 
         * @return Pose2d
         */
        public Pose2d getPose2d() {
                return m_odometry.getPoseMeters();
        }
        
        /** 
         * @param states
         */
        public void setAllStates(SwerveModuleState[] states){
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
m_backLeftModule.set(-states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
                updateOdometry(states);
                
        }
        public void resetOdometry(){
                // THIS MUST BE CALLED AFTER GYRO RESET
                m_odometry.resetPosition(Constants.auto.startingPos.DEFAULT_POS, getGyroscopeRotation());
        }

        
        /** 
         * @param resetPos
         */
        public void resetOdometry(Pose2d resetPos){
                m_odometry.resetPosition(resetPos, getGyroscopeRotation());
        }

        // Moves the modules into an "X" to prevent us from getting bullied and stops
        // the motors
        public void defense() {
                m_frontLeftModule.set(0, Math.toRadians(45));
                m_frontRightModule.set(0, Math.toRadians(-45));
                m_backLeftModule.set(0, Math.toRadians(-45));
                m_backRightModule.set(0, Math.toRadians(45));
        }

        
        /** 
         * @param goalPose
         * @param linearVelocity
         */
        public void trajectoryFollow(Pose2d goalPose, double linearVelocity) {

                // Calculate the velocities for the chassis
                ChassisSpeeds adjustedVelocities = follower.calculate(getPose2d(), goalPose, linearVelocity,
                        goalPose.getRotation());

                //SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(adjustedVelocities);
                drive(adjustedVelocities);
        }
        
        /** 
         * @param goalPose
         */
        public void trajectoryFollow(Pose2d goalPose) {

                // Calculate the velocities for the chassis
                ChassisSpeeds adjustedVelocities = follower.calculate(getPose2d(), goalPose, Constants.auto.follower.LINEAR_VELOCITY_DEFAULT,
                goalPose.getRotation());

                //SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(adjustedVelocities);
                drive(adjustedVelocities);
        }

        
        /** 
         * @return boolean
         */
        public boolean finishedMovement() {
                return follower.atReference();
        }

	
        /** 
         * @return SwerveDriveKinematics
         */
        public SwerveDriveKinematics getKinematics() {
			return m_kinematics;
	}

	
        /** 
         * @return boolean
         */
        public static boolean getFieldRelative() {
		return fieldRelative;
	}

	
        /** 
         * @return boolean
         */
        public static boolean getBrakeMode() {
		return brakeMode;
        }
        
        @Config
        /** 
         * @param x
         */
        public static void setFieldRelative(boolean x){
                fieldRelative = x;
        }

        @Config
        /** 
         * @param x
         */
        public static void setBrakeMode(boolean x){
                brakeMode = x;
        }

}
