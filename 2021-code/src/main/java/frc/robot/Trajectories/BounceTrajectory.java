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

public class BounceTrajectory {
    public static final Trajectory bounce0 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.8, 0.1)),
        new Pose2d(0.80, 1.0, Rotation2d.fromDegrees(90)),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0.80, 1.0, Rotation2d.fromDegrees(90)),
        new Pose2d(2.8, -1.62, Rotation2d.fromDegrees(180)),
        new Pose2d(3.4, 1.0, Rotation2d.fromDegrees(270))),
        NeutrinoTrajectoryConfigs.m_SlowReverseConfig ); 

    public static final Trajectory bounce2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.4, 1.0, Rotation2d.fromDegrees(270)),
        List.of(new Translation2d(3.5, -0.11)),
        new Pose2d(3.6, -1.62, Rotation2d.fromDegrees(285)), 
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.6, -1.62, Rotation2d.fromDegrees(285)),
        List.of(new Translation2d(5.6, -1.62)),
        new Pose2d(5.8, 0.6, Rotation2d.fromDegrees(285)), 
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce4 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(5.8, 0.6, Rotation2d.fromDegrees(285)),
        new Pose2d(6.8, 0.20, Rotation2d.fromDegrees(-180))), 
        NeutrinoTrajectoryConfigs.m_SlowReverseConfig);
}
