// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.util.Units;

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
public final class Constants {

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
            public static final double MODULE_STEER_OFFSET = -Math.toRadians(4.9);
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

    public static final class maximums {
        public static final class swerve {
            public static final double MAX_VEL_METERS = 6380.0 / 60.0 * SdsModuleConfigurations.MK3_FAST.getDriveReduction() * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;
            public static final double MAX_ANG_VEL_RAD = MAX_VEL_METERS /
            Math.hypot(Constants.dimensions.TRACKWIDTH / 2.0, Constants.dimensions.WHEELBASE / 2.0);
            public static final double MAX_VOLTAGE = 12.0;

        }
    }
}
