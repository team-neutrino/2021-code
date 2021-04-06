
package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class GalacticARedTrajectory
{
    public static final Trajectory galRedA = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1.25, 0), new Translation2d(3.22, -1), new Translation2d(2.5, 1.2), 
        new Translation2d(5.2, .5)),
        new Pose2d(7.5, .6, Rotation2d.fromDegrees(0)), NeutrinoTrajectoryConfigs.m_GalacticRedAForwardConfig);
}
