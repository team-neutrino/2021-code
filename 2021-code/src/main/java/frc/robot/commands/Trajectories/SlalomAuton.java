/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Trajectories.BounceTrajectory;
import frc.robot.Trajectories.SlalomTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;

public class SlalomAuton extends SequentialCommandGroup
{
    private DriveSubsystem m_Drive;
    private IntakePIDSubsystem m_Intake;

    public SlalomAuton(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake)
    {
        m_Drive = p_Drive;
        m_Intake = p_Intake;
        Trajectory m_slalom0 = SlalomTrajectory.slalom0;

        RamseteCommand slalom0Command = new RamseteCommand(m_slalom0, m_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0), m_Drive::tankDriveVolts, m_Drive);

        addCommands(
            new InstantCommand(() -> m_Intake.setAngle(39)).alongWith(
            slalom0Command),
            new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0)));
    }
}
