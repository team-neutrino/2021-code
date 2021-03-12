
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
        List.of(new Translation2d(4.31, -0.83), new Translation2d(5.68, 0.44), new Translation2d(7.45, -1.3)),
        new Pose2d(8.47, 0, Rotation2d.fromDegrees(0)), NeutrinoTrajectoryConfigs.m_FastForwardConfig);

}
