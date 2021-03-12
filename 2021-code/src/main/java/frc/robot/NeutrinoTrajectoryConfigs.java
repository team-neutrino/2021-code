/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;

public class NeutrinoTrajectoryConfigs
{
    private static final DifferentialDriveVoltageConstraint m_autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER,
            DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        DriveConstants.K_DRIVE_KINEMATICS, 10);

    public static final TrajectoryConfig m_SlowForwardConfig = new TrajectoryConfig(
        DriveConstants.K_MAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint);

    public static final TrajectoryConfig m_SlowReverseConfig = new TrajectoryConfig(
        DriveConstants.K_MAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint).setReversed(true);

    public static final TrajectoryConfig m_FastForwardConfig = new TrajectoryConfig(
        DriveConstants.K_MAAAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint);

    public static final TrajectoryConfig m_FastReverseConfig = new TrajectoryConfig(
        DriveConstants.K_MAAAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint).setReversed(true);

    public static final TrajectoryConfig m_TwixtForwardConfig = new TrajectoryConfig(
        DriveConstants.K_MAAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint);

    public static final TrajectoryConfig m_TwixtReverseConfig = new TrajectoryConfig(
        DriveConstants.K_MAAX_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint).setReversed(true);

    public static final TrajectoryConfig m_BounceForwardConfig = new TrajectoryConfig(
        DriveConstants.K_BOUNCE_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint);

    public static final TrajectoryConfig m_BounceReverseConfig = new TrajectoryConfig(
        DriveConstants.K_BOUNCE_SPEED_METERS_PER_SECOND,
        DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED).setKinematics(
            DriveConstants.K_DRIVE_KINEMATICS).addConstraint(m_autoVoltageConstraint).setReversed(true);

}
