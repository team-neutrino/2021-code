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
        List.of(new Translation2d(1.28, 0.1)), new Pose2d(1.28, 1.0, Rotation2d.fromDegrees(90)),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(1.28, 1.0, Rotation2d.fromDegrees(90)), new Pose2d(3.28, -1.62, Rotation2d.fromDegrees(180)),
            new Pose2d(3.88, 1.0, Rotation2d.fromDegrees(270))),
        NeutrinoTrajectoryConfigs.m_SlowReverseConfig);

    public static final Trajectory bounce2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.88, 1.0, Rotation2d.fromDegrees(270)), List.of(new Translation2d(3.98, -0.11)),
        new Pose2d(4.08, -1.62, Rotation2d.fromDegrees(285)), NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.08, -1.62, Rotation2d.fromDegrees(285)), List.of(new Translation2d(6.08, -1.62)),
        new Pose2d(6.28, 0.6, Rotation2d.fromDegrees(285)), NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory bounce4 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(6.28, 0.6, Rotation2d.fromDegrees(285)),
            new Pose2d(7.28, 0.20, Rotation2d.fromDegrees(-180))),
        NeutrinoTrajectoryConfigs.m_SlowReverseConfig);

    public static final Trajectory bounce5 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.8, 1.20, Rotation2d.fromDegrees(285)), List.of(new Translation2d(6.1, 0.90)),
        new Pose2d(6.4, 0.80, Rotation2d.fromDegrees(180)), NeutrinoTrajectoryConfigs.m_SlowForwardConfig);
}
