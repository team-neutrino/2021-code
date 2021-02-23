// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Trajectories.TenBallTrajectory;
import frc.robot.commands.TurretSetAngleCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TenBallAuton extends SequentialCommandGroup {
  /** Creates a new TenBallAuton. */

  public TenBallAuton(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake, TurretSubsystem p_Turret, ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper) {


    Trajectory m_tenBall0 = TenBallTrajectory.tenBall0;
    Trajectory m_tenBall1 = TenBallTrajectory.tenBall1;
    Trajectory m_tenBall2 = TenBallTrajectory.tenBall2;
    Trajectory m_tenBall3 = TenBallTrajectory.tenBall3;

    RamseteCommand tenBall0 = new RamseteCommand(
        m_tenBall0,
        p_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        p_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        p_Drive::tankDriveVolts,
        p_Drive
    );

    RamseteCommand tenBall1 = new RamseteCommand(
        m_tenBall1,
        p_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        p_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        p_Drive::tankDriveVolts,
        p_Drive
    );

    RamseteCommand tenBall2 = new RamseteCommand(
        m_tenBall2,
        p_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        p_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        p_Drive::tankDriveVolts,
        p_Drive
    );

    RamseteCommand tenBall3 = new RamseteCommand(
        m_tenBall3,
        p_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        p_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
  
        p_Drive::tankDriveVolts,
        p_Drive
    );

    addCommands(
      new InstantCommand(p_Turret::setLightOn),
      // new InstantCommand(p_Turret::startTimer),
      // new InstantCommand(() -> p_Turret.setAngle(70)).alongWith(
      new InstantCommand(p_Intake::setIntakeOn).alongWith(
        new SequentialCommandGroup(
          new ShootAuton(p_Shooter, p_Hopper, 5, 70000),
          tenBall0,
          new InstantCommand(() -> p_Drive.tankDriveVolts(0, 0)),
          new ShootAuton(p_Shooter, p_Hopper, 5, 70000), 
          tenBall1, 
          tenBall2,
          new InstantCommand(() -> p_Drive.tankDriveVolts(0, 0)),
          new ShootAuton(p_Shooter, p_Hopper, 5, 70000),
          tenBall3,
          new InstantCommand(() -> p_Drive.tankDriveVolts(0, 0)),
          new ShootAuton(p_Shooter, p_Hopper, 5, 70000))
      )
    );
  }
}
