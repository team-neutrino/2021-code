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
    List.of(new Translation2d(3, 0), new Translation2d(3.4, -1.5), new Translation2d(2.3, -1.3), new Translation2d(2.2, -.3),
    new Translation2d(4.5, 0), new Translation2d(6, 0), 
    //2nd curve
    new Translation2d(6.1, 1.4), new Translation2d(4.8, 1.5), 
    new Translation2d(4.7, 0), new Translation2d(5.2, 0), 
    //3rd curve
    new Translation2d(7.8, 1), new Translation2d(7.9, 0), new Translation2d(7.2, -1.4), 
    new Translation2d(3.3, 0), new Translation2d(1, -.5)),
    //endpoint
    new Pose2d(.8, -.5, Rotation2d.fromDegrees(180)),
    NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);
}
