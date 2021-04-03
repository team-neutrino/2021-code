
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
        List.of(new Translation2d(.8, -.22), 
        new Translation2d(2.3, -1.5)),
        new Pose2d(4, -.45, Rotation2d.fromDegrees(-18)), 
        NeutrinoTrajectoryConfigs.m_GalacticRedAForwardConfig);

    public static final Trajectory galRedA1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(4, -.45, Rotation2d.fromDegrees(-18)),
        new Pose2d(6.3, 4.9, Rotation2d.fromDegrees(-31))), 
        NeutrinoTrajectoryConfigs.m_GalacticRedAForwardConfig);
}
