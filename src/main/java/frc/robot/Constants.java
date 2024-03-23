// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.sorutil.ConstantAxis;
import frc.sorutil.ConstantButton;
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
        public static final double WIDTH = Units.inchesToMeters(28);
        public static final double LENGTH = Units.inchesToMeters(28);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
                new SwerveDriveKinematics(new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2));
    }

    public static final class Climb {
        // TODO: PLACEHOLDER PID
        public static final PidProfile PID_PROFILE = new PidProfile(0, 0, 0);
        public static final double CLIMB_CURRENT_LIMIT = 60.0;
        public static final double CLIMB_MAX_OUTPUT = 0.4;
    }
    public static final class Inouttake {
        // TODO: PLACEHOLDER PID
        public static final PidProfile PID_PROFILE = new PidProfile(0.01, 0, 0);
        public static final double OUTTAKE_CURRENT_LIMIT = 30.0;
        public static final double OUTTAKE_MAX_OUTPUT = 0.8;
        public static final double INTAKE_CURRENT_LIMIT = 30.0;
        public static final double INTAKE_MAX_OUTPUT = 0.8;
    }
    
    public static final class Vision {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
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
        public static final int ROLLER_INTAKE = 12;
        public static final int ROLLER_RELAY = 13;
        public static final int ROLLER_OUTTAKE_BOTTOM = 15;
        public static final int ROLLER_OUTTAKE_TOP = 14;

        public static final int ACTUAL_CLIMB_LEFT = 16;
        public static final int ACTUAL_CLIMB_RIGHT = 17;
    }
  
    public static final class Pivot {
        public static final PidProfile PID_PROFILE = new PidProfile(0.04, 0, 0.1);
        public static final double PIVOT_CURRENT_LIMIT = 30;
        public static final double PIVOT_MAX_OUTPUT = 0.8;

        // TODO: PLACEHOLDERS!
        public static final double INTAKE_ANGLE = 3.5;
        public static final double AMP_ANGLE = 90;
        public static final double SPEAKER_ANGLE = 15;
        public static final double STOW_ANGLE = 25;
    }
    public static final class Pneumatics {
        
    }

    public static final class Swerve {
        // TODO: SET THESE UP! Currently placeholders
        public static final PidProfile STEER_PROFILE = new PidProfile(0.03, 0, 0.1);
        public static final PidProfile POWER_PROFILE = new PidProfile(0.0002, 0.0, 0);

        public static final double SWERVE_POWER_CURRENT_LIMIT = 50.0;
        public static final double SWERVE_POWER_MAX_OUTPUT = 0.8;

        public static final double SWERVE_ROTATION_CURRENT_LIMIT = 30.0;
        public static final double SWERVE_ROTATION_MAX_OUTPUT = 0.7;

        public static final double DISTANCE_PER_REV = Units.inchesToMeters(4 * Math.PI);
        public static final double NEO_MAX_SPEED = 5600; // RPM the old speed was 5600
        public static final double MAX_WHEEL_SPEED = ((NEO_MAX_SPEED / 60) * DISTANCE_PER_REV);
        public static final double SWERVE_MAX_SPEED = 5.88; // m/s, was 6
        public static final double SWERVE_MAX_AUTO_SPEED = 0.2 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_PRECISION_SPEED = 0.1 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_ACCELERATION = 2; // m/s^2
        public static final double SWERVE_ROTATION_MAX_SPEED = Math.PI * 2; // rad/s
        public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI * 2 / 3; // rads/s^2

        public static final double SWERVE_DEADBAND = 0.0575;
        public static final double SWERVE_ROTATION_TOLERANCE = 3; // degrees
        public static final double SWERVE_SNAPPING_DEADBAND = 0.5; // 50%
    }

    public static final class Input {
        public static final ConstantAxis SWERVE_X_INPUT = new ConstantAxis(0, 1);
        public static final ConstantAxis SWERVE_Y_INPUT = new ConstantAxis(0, 0);
        public static final ConstantAxis SWERVE_ROTATION_INPUT = new ConstantAxis(0, 4);

        public static final ConstantButton shoot = new ConstantButton(0, 6);

        public static final ConstantButton stow = new ConstantButton(1, 6);
        public static final ConstantButton speaker = new ConstantButton(1, 5);
        public static final ConstantButton intake = new ConstantButton(1, 10);
        public static final ConstantButton amp = new ConstantButton(1, 8);

        public static final ConstantButton climbup = new ConstantButton(0, 2);
        public static final ConstantButton climbdown = new ConstantButton(0, 3);

        public static final ConstantButton SWERVE_FACE_N = new ConstantButton(0, 0);
        public static final ConstantButton SWERVE_FACE_E = new ConstantButton(0, 90);
        public static final ConstantButton SWERVE_FACE_S = new ConstantButton(0, 180);
        public static final ConstantButton SWERVE_FACE_W = new ConstantButton(0, 270);
    }
    public static final class Auto {
        public static final double AUTO_INTAKE_TIME = 2;
        public static final double AUTO_OUTTAKE_TIME = 0.5;
    }
    public static final class PoseEstimation {
        public static final Matrix<N3, N1> POSE_GYRO_STD = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> POSE_VISION_STD = VecBuilder.fill(0.1, 0.1, 0.1);
    }

    public static final class Scoring {
        public static final double AUTO_SWERVE_MAX_VELOCITY = 4; // Meters per second
        public static final double AUTO_SWERVE_MAX_ACCELERATION = 2.5; // Meters per second


        //TODO: Fix PID Values
        public static final PIDController X_CONTROLLER = new PIDController(0, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(0, 0, 0);
        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(AUTO_SWERVE_MAX_VELOCITY, AUTO_SWERVE_MAX_ACCELERATION));
        public static final double TRAJECTORY_SAMPLE_TIME = 0; // seconds
    }
}
