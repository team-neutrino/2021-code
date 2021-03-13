package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class BarrelRaceTrajectory {
    public static final Trajectory barrelRace0 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
    List.of(new Translation2d(3.5, -0.8), new Translation2d(3.4, -1.5), new Translation2d(2.3, 1.7)), 
    new Pose2d(3.9, 0.04, Rotation2d.fromDegrees(-60)),
    NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);

    public static final Trajectory barrelRace1 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.9, 0.04, new Rotation2d(-60)),
    List.of(new Translation2d(6, 0.8), new Translation2d(3.1, 1.2), new Translation2d(4.9, 0.1)), 
    new Pose2d(7.5, -.4, Rotation2d.fromDegrees(0)),
    NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);

    public static final Trajectory barrelRace2 = TrajectoryGenerator.generateTrajectory(new Pose2d(7.4, -0.4, new Rotation2d(0)),
    List.of(new Translation2d(7.3, -1.8), new Translation2d(6, 1.5)), new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
    NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);
}
