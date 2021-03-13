package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.NeutrinoTrajectoryConfigs;
import frc.robot.subsystems.TroubleshootingSubsystem;

public class TeleopTrajectory {
    public static double p_drive_distance = TroubleshootingSubsystem.m_drive_distance.getNumber(0).doubleValue();

    public static int calibrate_count = 0;

    /*public static double angle_calibrate() 
    {
        return 2*(double)calibrate_count;
    }*/

    public static final Trajectory toPort = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(p_drive_distance, 0, new Rotation2d(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_TwixtReverseConfig);

    public static final Trajectory fromPort = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(p_drive_distance, 0, Rotation2d.fromDegrees(0))),
        NeutrinoTrajectoryConfigs.m_TwixtForwardConfig);
}
