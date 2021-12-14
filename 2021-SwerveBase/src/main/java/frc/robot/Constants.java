// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants implements Loggable {

    public static final class dimensions {
        public static final double TRACKWIDTH = Units.inchesToMeters(25);
        public static final double WHEELBASE = Units.inchesToMeters(23.75);
    }

    public static final class modInfo {
        public static final class flMod {
            public static final int MODULE_DRIVE_MOTOR = 4;
            public static final int MODULE_STEER_MOTOR = 5;
            public static final int MODULE_STEER_ENCODER = 2;
            public static final double MODULE_STEER_OFFSET = -Math.toRadians(12.75);
        }

        public static final class frMod {
            public static final int MODULE_DRIVE_MOTOR = 6;
            public static final int MODULE_STEER_MOTOR = 7;
            public static final int MODULE_STEER_ENCODER = 3;
            public static final double MODULE_STEER_OFFSET = -Math.toRadians(349);
        }

        public static final class blMod {
            public static final int MODULE_DRIVE_MOTOR = 2;
            public static final int MODULE_STEER_MOTOR = 3;
            public static final int MODULE_STEER_ENCODER = 1;
            public static final double MODULE_STEER_OFFSET = -Math.toRadians(-25.5);
        }

        public static final class brMod {
            public static final int MODULE_DRIVE_MOTOR = 0;
            public static final int MODULE_STEER_MOTOR = 1;
            public static final int MODULE_STEER_ENCODER = 0;
            public static final double MODULE_STEER_OFFSET = -Math.toRadians(263.3);
        }
    }

    public static final class swerve {
        // ORDER: FL FR BL BR
        @Log
        public static final double MAX_VEL_METERS = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_FAST.getDriveReduction()
                * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
        @Log
        public static final double MAX_ANG_VEL_RAD = MAX_VEL_METERS
                / Math.hypot(Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0);
        @Log        
        public static final double MAX_VOLTAGE = 12.0;
        @Log
        public static final double MAX_ANG_ACCEL = 8 * Math.PI;
        public static final boolean feildRelativeOn = true;
        public static final boolean brakeModeOn = false;
        

    }

    public static final class outputs {
        public static final double strafe = .7;
        public static final double turnRate = 1;
    }

    public static final class auto {

        /*
         * public static final Matrix<N3, N1> POSE_STD_DEV = new MatBuilder<>(Nat.N5(),
         * Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard
         * deviations. X, Y, theta. public static final Matrix<N3, N1> ENCODER_GYRO_DEV
         * = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local
         * measurement standard deviations. Left encoder, right encoder, gyro. public
         * static final Matrix<N3, N1> VISION_DEVIATION = new MatBuilder<>(Nat.N3(),
         * Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations.
         * X, Y, and theta.
         */
        public static final class follower {
            @Log
            private static final double MAX_ANG_VEL_RAD_AUTO = 8 * Math.PI;
            public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(
                    MAX_ANG_VEL_RAD_AUTO, swerve.MAX_ANG_ACCEL);
            @Log
            public static final PIDController X_PID_CONTROLLER = new PIDController(5.25, 1, .4);
            @Log
            public static final PIDController Y_PID_CONTROLLER = new PIDController(5.25, 1, .4);
            @Log
            public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(.13, 0, .39,
                    ROT_PROFILE);
            // DRIVING DEFAULT IS 5
            public static final double LINEAR_VELOCITY_DEFAULT = 1;
            // MUST SET KINEMATICS, see documentation
            public static final TrajectoryConfig T_CONFIG = new TrajectoryConfig(LINEAR_VELOCITY_DEFAULT,
                    swerve.MAX_ANG_VEL_RAD);
        }

        public static final class startingPos {
            public static final Pose2d DEFAULT_POS = new Pose2d();
        }

    }
}
