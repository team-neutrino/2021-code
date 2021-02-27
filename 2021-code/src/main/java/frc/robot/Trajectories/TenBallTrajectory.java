
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
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), List.of(new Translation2d(4.6, 0)),
        new Pose2d(4.78, -3.87, Rotation2d.fromDegrees(-156.2)), NeutrinoTrajectoryConfigs.m_FastForwardConfig);

    public static final Trajectory tenBall1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(4.78, -3.87, Rotation2d.fromDegrees(-156.2)),
            new Pose2d(4.52, -4.82, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

    public static final Trajectory tenBall2 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(4.52, -4.82, Rotation2d.fromDegrees(0)),
            new Pose2d(4.25, -7.74, Rotation2d.fromDegrees(-90))),
        NeutrinoTrajectoryConfigs.m_SlowForwardConfig);
}
