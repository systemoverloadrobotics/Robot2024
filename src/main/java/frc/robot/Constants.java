// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.sorutil.motor.PidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    public static final String PROJECT_NAME = "Robot2024";

    // Periodic timing of the robot, WPILib default is 0.02 (20ms)
    public static final Double ROBOT_PERIOD = 0.02; // 20 ms

    // Configure the power module used on the robot
    public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final class RobotDimensions {
        // TODO: replace these with actual dimensions
        public static final double WIDTH = Units.inchesToMeters(30);
        public static final double LENGTH = Units.inchesToMeters(30);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
                new SwerveDriveKinematics(new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2));
    }

    public static final class Climb {
        public static final PidProfile PID_PROFILE = new PidProfile(0, 0, 0);
        public static final double CLIMB_CURRENT_LIMIT = 40.0;
        public static final double CLIMB_MAX_OUTPUT = 0.5;
    }
    public static final class Inouttake {
        public static final PidProfile PID_PROFILE = new PidProfile(0, 0, 0);
        public static final double OUTTAKE_CURRENT_LIMIT = 30.0;
        public static final double OUTTAKE_MAX_OUTPUT = 0.8;
        public static final double INTAKE_CURRENT_LIMIT = 30.0;
        public static final double INTAKE_MAX_OUTPUT = 0.8;
    }
    
    public static final class Vision {
        // Camera location from the center of the robot
        public static final double CAMERA_POSITION_X = 0; // Meters
        public static final double CAMERA_POSITION_Y = 0.5; // Meters
        public static final double CAMERA_POSITION_Z = 0; // Meters
        public static final double CAMERA_ROTATION_ROLL = 0; // Radians
        public static final double CAMERA_ROTATION_PITCH = 0; // Radians
        public static final double CAMERA_ROTATION_YAW = 0; // Radians

        public static final Translation3d CAMERA_POSITION =
                new Translation3d(CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z);
        public static final Rotation3d CAMERA_ROTATION =
                new Rotation3d(CAMERA_ROTATION_ROLL, CAMERA_ROTATION_PITCH, CAMERA_ROTATION_YAW);

        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT;

        static {
            AprilTagFieldLayout temp = null;
            try {
                temp = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2023-chargedup.json");
            } catch (IOException e) {
                e.printStackTrace();
            }
            TAG_FIELD_LAYOUT = temp;
        }
    }

    public static final class Motor {
        // Motor indexes + configs here
        public static final int SWERVE_FRONT_LEFT_POWER = 1;
        public static final int SWERVE_FRONT_LEFT_STEER = 2;
        public static final int SWERVE_FRONT_RIGHT_POWER = 3;
        public static final int SWERVE_FRONT_RIGHT_STEER = 4;
        public static final int SWERVE_BACK_LEFT_POWER = 5;
        public static final int SWERVE_BACK_LEFT_STEER = 6;
        public static final int SWERVE_BACK_RIGHT_POWER = 7;
        public static final int SWERVE_BACK_RIGHT_STEER = 8;

        public static final int CLIMB_LEFT = 9;
        public static final int CLIMB_RIGHT = 10;

        public static final int ROLLER_INTAKE_BOTTOM = 11;
        public static final int ROLLER_INTAKE_TOP = 12;
        public static final int ROLLER_OUTTAKE_BOTTOM = 13;
        public static final int ROLLER_OUTTAKE_TOP = 14;


    }

    public static final class Pneumatics {
        
    }

    public static final class Swerve {
        // TODO: SET THESE UP! Currently placeholders
        public static final PidProfile STEER_PROFILE = new PidProfile(0.03, 0, 0.1);
        public static final PidProfile POWER_PROFILE = new PidProfile(0.0002, 0.0, 0);

        public static final double SWERVE_POWER_CURRENT_LIMIT = 60.0;
        public static final double SWERVE_POWER_MAX_OUTPUT = 0.8;

        public static final double SWERVE_ROTATION_CURRENT_LIMIT = 40.0;
        public static final double SWERVE_ROTATION_MAX_OUTPUT = 0.7;

        public static final double DISTANCE_PER_REV = Units.inchesToMeters(4 * Math.PI);
        public static final double NEO_MAX_SPEED = 5820; // RPM the old speed was 5600
        public static final double MAX_WHEEL_SPEED = ((NEO_MAX_SPEED / 60) * DISTANCE_PER_REV);
        public static final double SWERVE_MAX_SPEED = 5; // m/s, was 6
        public static final double SWERVE_MAX_AUTO_SPEED = 0.2 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_PRECISION_SPEED = 0.1 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_ACCELERATION = 2; // m/s^2
        public static final double SWERVE_ROTATION_MAX_SPEED = Math.PI * 2; // rad/s
        public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI * 2 / 3; // rads/s^2

        public static final double SWERVE_DEADBAND = 0.05;
        public static final double SWERVE_ROTATION_TOLERANCE = 5; // degrees
        public static final double SWERVE_SNAPPING_DEADBAND = 0.5; // 50%
    }

    public static final class Input {
        
    }
    public static final class Auto {
        
    }
    public static final class PoseEstimation {
        
    }
}
