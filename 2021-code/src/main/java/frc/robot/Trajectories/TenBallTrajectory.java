
package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class TenBallTrajectory
{

    public static final Trajectory tenBall0 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(4.6, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);

    public static final Trajectory tenBallHalf = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(4.6, 0, Rotation2d.fromDegrees(-90)), new Pose2d(4, -2.6, Rotation2d.fromDegrees(-147))),
        NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);

    public static final Trajectory tenBall1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4, -2.6, Rotation2d.fromDegrees(-147)), List.of(new Translation2d(2.7, -3.9)),
        new Pose2d(4.3, -3.6, Rotation2d.fromDegrees(25)), NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory tenBall2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.3, -3.76, Rotation2d.fromDegrees(25)), List.of(new Translation2d(3.7, -5.5)),
        new Pose2d(3.7, -6.4, Rotation2d.fromDegrees(-90)), NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
