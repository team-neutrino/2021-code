// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Trajectories.ExampleTrajectory;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceAuto extends SequentialCommandGroup {
  /** Creates a new BounceAuto. */
  private DriveSubsystem m_Drive;
  public BounceAuto(DriveSubsystem p_Drive) {
    m_Drive = p_Drive;
    Trajectory m_bounce0 = ExampleTrajectory.bounce0;
    Trajectory m_bounce1 = ExampleTrajectory.bounce1;
    Trajectory m_bounce2 = ExampleTrajectory.bounce2;
    Trajectory m_bounce3 = ExampleTrajectory.bounce3;
    Trajectory m_bounce4 = ExampleTrajectory.bounce4;
    Trajectory m_bounce5 = ExampleTrajectory.bounce5;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteCommand bounce0Command = new RamseteCommand(
        m_bounce0,
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
    RamseteCommand bounce1Command = new RamseteCommand(
        m_bounce1,
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
    RamseteCommand bounce2Command = new RamseteCommand(
        m_bounce2,
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
    RamseteCommand bounce3Command = new RamseteCommand(
        m_bounce3,
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
    RamseteCommand bounce4Command = new RamseteCommand(
        m_bounce4,
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
    RamseteCommand bounce5Command = new RamseteCommand(
        m_bounce5,
        m_Drive::getPose,
        new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
        new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.DriveConstants.K_DRIVE_KINEMATICS,
        m_Drive::getWheelSpeeds,
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
        new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0), 
        m_Drive::tankDriveVolts, m_Drive
    );
    addCommands(bounce0Command, bounce1Command, bounce2Command, bounce3Command, bounce4Command, bounce5Command, new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0)));
  }
}
