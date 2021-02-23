// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Trajectories.BounceTrajectory;
import frc.robot.Trajectories.TenBallTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TenBallAuton extends SequentialCommandGroup {
  /** Creates a new TenBallAuton. */
  DriveSubsystem m_Drive;
  IntakePIDSubsystem m_Intake;
  TurretSubsystem m_Turret;
  public TenBallAuton(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake, TurretSubsystem p_Turret) {
    m_Drive = p_Drive;
    m_Intake = p_Intake;
    m_Turret = p_Turret;

    Trajectory m_tenBall0 = TenBallTrajectory.tenBall0;
    Trajectory m_tenBall1 = TenBallTrajectory.tenBall1;
    Trajectory m_tenBall2 = TenBallTrajectory.tenBall2;

    RamseteCommand tenBall0 = new RamseteCommand(
        m_tenBall0,
        m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        m_Drive::tankDriveVolts,
        m_Drive
    );

    RamseteCommand tenBall1 = new RamseteCommand(
        m_tenBall1,
        m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        m_Drive::tankDriveVolts,
        m_Drive
    );

    RamseteCommand tenBall2 = new RamseteCommand(
        m_tenBall2,
        m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        m_Drive::tankDriveVolts,
        m_Drive
    );

    addCommands(
      new InstantCommand(m_Turret::setLightOn),
      new InstantCommand(m_Intake::setIntakeOn).alongWith(
        new SequentialCommandGroup(
          //shoot
          tenBall0,
          //shoot 
          tenBall1, 
          tenBall2,
          //shoot
          new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0)))
      )
    );
  }
}
