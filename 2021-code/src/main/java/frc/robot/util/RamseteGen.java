// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.*;

/** Add your docs here. */
public class RamseteGen {
    Trajectory trajectory = new Trajectory();
    private final DriveSubsystem m_Drive = new DriveSubsystem();
    
    private String trajectoryJSON;

    public RamseteGen(String p_trajectoryJSON) {
        trajectoryJSON = p_trajectoryJSON;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        
    }

    public RamseteCommand trajectoryRamsete = new RamseteCommand(
        trajectory, 
        m_Drive::getPose, 
        new RamseteController(DriveConstants.K_RAMSETE_B, 
                                DriveConstants.K_RAMSETE_ZETA), 
        new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
                                    DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        DriveConstants.K_DRIVE_KINEMATICS, 
        m_Drive::getWheelSpeeds, 
        new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0), 
        new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
        m_Drive::tankDriveVolts, 
        m_Drive
    );
    
    
}
