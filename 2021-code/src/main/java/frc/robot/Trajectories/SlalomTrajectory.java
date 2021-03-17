/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Trajectories;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.NeutrinoTrajectoryConfigs;
import frc.robot.Constants.FieldConstants;

public class SlalomTrajectory
{

    public static final Trajectory slalom0 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(FieldConstants.getX(1.5), FieldConstants.getY('e', true), new Rotation2d(-30)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(FieldConstants.getX(6), FieldConstants.getY('c', false)), //highest point
            new Translation2d(FieldConstants.getX(8.5), FieldConstants.getY('d', true)), //rightmost intersection
            new Translation2d(FieldConstants.getX(10), FieldConstants.getY('e', false)),
            new Translation2d(FieldConstants.getX(11), FieldConstants.getY('d', false)),
            new Translation2d(FieldConstants.getX(10), FieldConstants.getY('c', false)),
            new Translation2d(FieldConstants.getX(9), FieldConstants.getY('d', false)), //right most intersection, after circle
            new Translation2d(FieldConstants.getX(6), FieldConstants.getY('e', false)),
            new Translation2d(FieldConstants.getX(3.5), FieldConstants.getY('e', true))), 
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(FieldConstants.getX(2), FieldConstants.getY('c', true), new Rotation2d(30)),
        // Pass config
        NeutrinoTrajectoryConfigs.m_FastForwardConfig);
}
