package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class EMCCAutonTrajectory {
    



    public static final Trajectory EMCC1 = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);

        
}
