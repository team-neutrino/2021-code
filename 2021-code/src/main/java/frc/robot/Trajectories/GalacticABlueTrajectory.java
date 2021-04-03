
package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class GalacticABlueTrajectory
{
    public static final Trajectory galBlueA = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(3.6, -1.6), new Translation2d(3, 0.9), new Translation2d(6, -.2)),
        new Pose2d(7.6, 0, Rotation2d.fromDegrees(0)), NeutrinoTrajectoryConfigs.m_GalacticRedAForwardConfig);
}
