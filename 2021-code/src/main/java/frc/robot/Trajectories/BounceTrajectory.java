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

public class BounceTrajectory
{
    //I added 2.48 to every x value because the robot went backwards by 19 inches
    public static final Trajectory bounce0 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.9, 0.1)), new Pose2d(0.9, 1.3, Rotation2d.fromDegrees(90)),
        NeutrinoTrajectoryConfigs.m_BounceForwardConfig);

    public static final Trajectory bounce1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.9, 1.3, Rotation2d.fromDegrees(90)),
        List.of(new Translation2d(1.4, 0.2), new Translation2d(1.9, -1), new Translation2d(2.6, -1.6),
            new Translation2d(3.25, -0.9)),
        new Pose2d(3.25, 1.3, Rotation2d.fromDegrees(270)), NeutrinoTrajectoryConfigs.m_BounceReverseConfig);

    public static final Trajectory bounce2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.25, 1.3, Rotation2d.fromDegrees(270)),
        List.of(new Translation2d(3.5, -0.9), new Translation2d(5.0, -1.8), new Translation2d(6, -1.1)),
        new Pose2d(5.75, 0.75, Rotation2d.fromDegrees(90)), NeutrinoTrajectoryConfigs.m_BounceForwardConfig);

    public static final Trajectory bounce3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.75, 0.75, Rotation2d.fromDegrees(90)), List.of(new Translation2d(6.3, -0.6)),
        new Pose2d(8, -0.85, Rotation2d.fromDegrees(180)), NeutrinoTrajectoryConfigs.m_BounceReverseConfig);
}
