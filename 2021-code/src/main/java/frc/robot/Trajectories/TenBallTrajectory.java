package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;

public class TenBallTrajectory {
    /*public static final Trajectory tenBall0 = TrajectoryGenerator.generateTrajectory( //first trench run
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(4.3, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);*/

        /*public static final Trajectory tenBall0 = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(4.6, 0, Rotation2d.fromDegrees(0))),
                NeutrinoTrajectoryConfigs.m_FastForwardConfig);*/

                public static final Trajectory tenBall0 = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    List.of(new Translation2d(4.6, 0)),
                    new Pose2d(4.55, -1.24, Rotation2d.fromDegrees(-120)), 
                    NeutrinoTrajectoryConfigs.m_FastForwardConfig);
        
        /*public static final Trajectory tenBall1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(4.6, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(4.0, -1.7)),
            new Pose2d(3.2, -2.6, Rotation2d.fromDegrees(-140)), 
            NeutrinoTrajectoryConfigs.m_SlowForwardConfig);*/
        
            public static final Trajectory tenBall1 = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(4.55, -1.24, Rotation2d.fromDegrees(-120)), new Pose2d(3.2, -2.75, Rotation2d.fromDegrees(-135))),
                NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

            public static final Trajectory tenBall2 = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(3.2, -2.75, Rotation2d.fromDegrees(-130)), new Pose2d(3.35, -3.19, Rotation2d.fromDegrees(-40))),
                NeutrinoTrajectoryConfigs.m_SlowForwardConfig);

        

    /*public static final Trajectory tenBall1 = TrajectoryGenerator.generateTrajectory( //rotate in-place for generator 
        List.of(new Pose2d(4.3, 0, new Rotation2d(0)), new Pose2d(4.3, 0, Rotation2d.fromDegrees())),
        NeutrinoTrajectoryConfigs.m_FastReverseConfig);*/
}
