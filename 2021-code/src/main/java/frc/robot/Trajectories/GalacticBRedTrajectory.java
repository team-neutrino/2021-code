/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.NeutrinoTrajectoryConfigs;

public class GalacticBRedTrajectory
{

    /*public static final Trajectory sixBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(4.6, 0, new Rotation2d(0))),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);*/

    public static final Trajectory galRedB = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(120), new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)), 
            new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)),
            new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(120), new Rotation2d(0)),
        // Pass config
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
