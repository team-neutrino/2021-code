/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class EightBallTrajectory {

    public static final Trajectory eightBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2.9, -2.1, Rotation2d.fromDegrees(-60))),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory eightBall1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(2.9, -2.1, Rotation2d.fromDegrees(-60)), new Pose2d(1.6, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_SlowReverseConfig);

    public static final Trajectory eightBall2 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(1.6, 0, Rotation2d.fromDegrees(0)), new Pose2d(4.6, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);
}
