/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */

public final class Constants
{

    public static final class DriveConstants
    {

        public static final double KS_VOLTS = 0.177;
        public static final double KV_VOLT_SECONDS_PER_METER = 3.21;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.444;
        public static final double KP_DRIVE_VEL = 2.15;
        public static final double K_TRACK_WIDTH_METERS = 0.7;
        public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(
            K_TRACK_WIDTH_METERS);

        public static final double K_MAX_SPEED_METERS_PER_SECOND = 1;
        public static final double K_MAAX_SPEED_METERS_PER_SECOND = 1.5;
        public static final double K_MAAAX_SPEED_METERS_PER_SECOND = 2.0;
        public static final double K_BOUNCE_SPEED_METERS_PER_SECOND = 3;
        public static final double K_GALACTICREDA_SPEED_METERS_PER_SECOND = 3.5;
        public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 6.8;
        public static final double K_BARREL_RACE_SPEED_METERS_PER_SECOND = 3.0;
        public static final double K_SLALOM_SPEED_METERS_PER_SECOND = 3.5;
        public static final double K_RAMSETE_B = 2;
        public static final double K_RAMSETE_ZETA = 0.7;

        public static final double K_GEAR_RATIO = 33.0 / 340.0;
        public static final double K_WHEEL_CIRCUMFERENCE = Math.PI * 0.127;
        //convert NEO encoder RPM to wheel meters/second
        public static final double K_DRIVE_ENCODER_CONVERSION = (K_GEAR_RATIO * K_WHEEL_CIRCUMFERENCE);
    }

    public static final class IntakeConstants
    {
        public static final double INTAKE_MOTOR_POWER = -1;
        public static final double OUTTAKE_MOTOR_POWER = 1;
        public static final double ARM_UP_ANGLE = 85;
        public static final int ENCODER_PORT = 8;

        public static final double KP = 0.015;
        public static final double KI = 0.0;
        public static final double KD = 0.0001;
        public static final double POSITION_MULTIPLIER = 360;
        public static final double LEFT_TRIGGER_THRESHOLD = 0.5;
        public static final double RIGHT_TRIGGER_THRESHOLD = 0.5;
    }

    public static final class ControllerPorts
    {
        public static final int XBOX_CONTROLLER_PORT = 2;
    }

    public static final class JoystickConstants
    {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK__PORT = 1;
        public static final int RIGHT_Z_PORT = 2;
        public static final double DEADZONE_SIZE = 0.1;
        public static final double JOYSTICK_CURVE = 1.0;
        public static final int RIGHT_JOYSTICK_TOP_PORT = 3;
    }

    public static final class ShooterConstants
    {

        public static final double WHEEL_P = 0.04;
        public static final double WHEEL_I = 0;
        public static final double WHEEL_D = 2;
        public static final double WHEEL_F = 0.008;
        public static final double SHOOTER_CONVERSION = 0;
        public static final int green = 69500;
        public static final int blue = 66000;
        public static final int red = 69100;
        public static final int yellow = 60000;
    }

    public static final class HoodConstants
    {
        public static final double HOOD_ANGLE_DOWN = 130;
        public static final double HOOD_ANGLE_UP = 70;
    }

    public static final class CanId
    {

        public static final int MOTOR_CONTROLLER_DRIVER_LEFT1 = 1;
        public static final int MOTOR_CONTROLLER_DRIVER_LEFT2 = 2;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT1 = 3;
        public static final int MOTOR_CONTROLLER_DRIVER_RIGHT2 = 4;
        public static final int MOTOR_CONTROLLER_CLIMBER = 5;
        public static final int MOTOR_CONTROLLER_HOPPER = 6;
        public static final int MOTOR_CONTROLLER_TURRET = 7;
        public static final int MOTOR_CONTROLLER_CLIMBERWINCH = 8;
        public static final int MOTOR_CONTROLLER_TOWER = 10;
        public static final int MOTOR_CONTROLLER_SHOOTERWHEEL3 = 11;
        public static final int MOTOR_CONTROLLER_SHOOTERWHEEL2 = 12;
        public static final int MOTOR_CONTROLLER_INTAKE_POSITION = 13;
        public static final int MOTOR_CONTROLLER_SHOOTERWHEEL = 14;
        public static final int MOTOR_CONTROLLER_INTAKE_FEED = 15;
    }

    public static final class ClimberConstants
    {
        public static final double CLIMBER_MOTOR_POWER_UP = .4;
        public static final double CLIMBER_MOTOR_POWER_DOWN = .1;
        public static final double CLIMBER_MOTOR_WINCHPOWER = -1;
        public static final double CLIMBER_BAR_HEIGHT = 0;
        public static final double CLIMBER_FULL_EXTEND = 40000;
    }
    public static final class HopperConstants
    {
        public static final double HOPPER_MOTOR_POWER = 0.8;
        public static final double HOPPER_MOTOR_POWER_REVERSE = -0.5;
        public static final int HOPPER_TOP_BEAMBREAK = 7;
        public static final int HOPPER_BOT_BEAMBREAK = 9;
    }

    public static final class VisionConstants
    {
        public static final double CAMERA_HEIGHT = 37.129;
        public static final double CAMERA_ANGLE = 25.0;
        public static final double TARGET_HEIGHT = 89.688;
        public static final double SCAN_SPEED = 0.5;
        public static final double TRACKING_KP = 0.1;
        public static final double TRACKING_CONSTANT_OFFSET = 0.05;
        public static final double SCAN_DIRECTION_SWITCH_RESET_THRESHOLD = 160.0;
        public static final double TURRET_ANGLE_TOLERANCE = 1.0;
    }

    public static final class TurretConstants
    {
        public static final double TURRET_OFFSET_ANGLE = -128;
        public static final double DEGREE_BOUNDS = 180.0;
        public static final double ANGLE_SCALE = (7.0 * 360.0) / (27.0 * 1024.0);
        public static final double kP = 0.10;
    }

    public static final class DistanceCalculatorConstant
    {
        public static final double ANGLE_TWO = 23;
        public static final double ANGLE_ = 23;
        public static final double LIMELIGHT_HEIGHT = 38;
        public static final double POWER_PORT_HEIGHT = 89.75;
        public static final double FINAL_HEIGHT = POWER_PORT_HEIGHT - LIMELIGHT_HEIGHT;
    }
    public static final class AutonSelectorConstant
    {
        //Underestimations to create a range
        public static final double RED_A = -5; //tx = -1;
        public static final double RED_A_TX1 = -.5; //tx1 = -.28
        public static final double RED_B = 16; //17
        public static final double BLUE_B = 11; //12.9
        public static final double BLUE_A = 21; //23.7
    }
}
