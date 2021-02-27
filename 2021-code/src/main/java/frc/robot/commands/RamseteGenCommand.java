// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RamseteGenCommand extends SequentialCommandGroup
{
    /** Creates a new RamseteGenCommand. */
    private DriveSubsystem m_Drive;
    private String m_trajectoryJSON;
    public RamseteGenCommand(DriveSubsystem p_Drive, String p_trajectoryJSON)
    {
        m_Drive = p_Drive;
        m_trajectoryJSON = p_trajectoryJSON;

        m_Drive.initAuton();

        Trajectory trajectory = new Trajectory();
        try
        {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(m_trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch (IOException ex)
        {
            DriverStation.reportError("Unable to open trajectory: " + m_trajectoryJSON, ex.getStackTrace());
        }

        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

            m_Drive::tankDriveVolts, m_Drive);

        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(ramseteCommand.andThen(() -> m_Drive.tankDriveVolts(0, 0)));
    }
}
