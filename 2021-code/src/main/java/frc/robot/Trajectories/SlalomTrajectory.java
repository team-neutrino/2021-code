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
import frc.robot.NeutrinoTrajectoryConfigs;

public class SlalomTrajectory
{
    //I added 2.48 to every x value because the robot went backwards by 19 inches
    public static final Trajectory slalom0 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1.25, 0.93), //first intersection
            new Translation2d(2.38, 1.9), // top 1st
            new Translation2d(5.1, 1.6), // top 2nd
            //new Translation2d(6.3, 0.7), // second intersection, 1st time
            new Translation2d(6.9, -0.2), // circle bottom 
            new Translation2d(7.9, 0.8), // circle right
            new Translation2d(7.3, 1.5), // circle top
            new Translation2d(5.5, -0.3), // second intersection, 2nd time
            new Translation2d(2.1, -0.4)
            ), 
        new Pose2d(0.6, 0.7, Rotation2d.fromDegrees(180)),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
