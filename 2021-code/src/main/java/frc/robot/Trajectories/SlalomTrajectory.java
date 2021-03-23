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
            new Translation2d(1.2, 0.93), // first intersection
            new Translation2d(2.35, 1.9), // top 1st
            new Translation2d(5, 1.9), // top 2nd
            //new Translation2d(6.4, 0.57), // 2nd intersection, 1st time
            new Translation2d(6.7, 0), // circle bottom, y might need to b 0?
            new Translation2d(7.7, 0.5), // circle right
            new Translation2d(7.2, 1.8), // circle top
            new Translation2d(6.4, 0.6), //2nd intersection, 2nd time
            new Translation2d(5.2, -0.4), //bottom 1st pt
            new Translation2d(4.6, -.1), //when it goes out of bounds, bottom middle
            new Translation2d(2.2, 0), //bottom 2nd pt
            new Translation2d(1.6, 0.7) //1st interesection 2nd time
            ), 
        new Pose2d(0.6, 1.65, Rotation2d.fromDegrees(180)),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
