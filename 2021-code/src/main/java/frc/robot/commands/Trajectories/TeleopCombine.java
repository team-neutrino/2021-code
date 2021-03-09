// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Trajectories.TeleopTrajectory;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TeleopCombine extends CommandBase {
  private DriveSubsystem m_Drive;

  private Trajectory m_fromPort = TeleopTrajectory.fromPort;
  private Trajectory m_toPort = TeleopTrajectory.toPort;

  private Command m_commandFromPort;
  private Command m_commandToPort;

  /** Creates a new TeleopCombine. */
  public TeleopCombine(DriveSubsystem p_Drive) {
    addRequirements(p_Drive);
    m_Drive = p_Drive;

    //Teleop2 m_teleop2 = new Teleop2(p_Drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RamseteCommand fromPortCommand = new RamseteCommand(m_fromPort, m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
            Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
            Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

        m_Drive::tankDriveVolts, m_Drive);

    TeleopTrajectory.calibrate_count++;
    
    m_commandFromPort = new SequentialCommandGroup(fromPortCommand, new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0)));
    m_commandFromPort.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RamseteCommand toPortCommand = new RamseteCommand(m_toPort, m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
            Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
            Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS, m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

        m_Drive::tankDriveVolts, m_Drive);
    m_commandToPort = new SequentialCommandGroup(toPortCommand, new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0)));
    m_commandToPort.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
