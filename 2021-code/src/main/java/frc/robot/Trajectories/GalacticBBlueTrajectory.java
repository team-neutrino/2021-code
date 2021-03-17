
package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class GalacticBBlueTrajectory
{
    public static final Trajectory galBlueB = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(0.95, 0.7), new Translation2d(2.6, -0.54), new Translation2d(4.1, 0.70)),
        new Pose2d(8.20, 7, Rotation2d.fromDegrees(0)), NeutrinoTrajectoryConfigs.m_GalacticRedAForwardConfig);

}