package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class TeleopTrajectory {
    public static final Trajectory toPort = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);

    public static final Trajectory fromPort = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(2, 0, new Rotation2d(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
