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
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.SixBallTrajectory;
import frc.robot.Trajectories.TenBallTrajectory;
import frc.robot.commands.TurretSetAngleCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustShoot extends SequentialCommandGroup
{
    /** Creates a new TenBallAuton. */

    public JustShoot(TurretSubsystem p_Turret, ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper, DriveSubsystem p_Drive, double angle)
    {
      
      Trajectory trajectory = SixBallTrajectory.sixBall0;

      PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
      PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);

      RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);

      SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
          DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

      RamseteCommand sixBallTraj0 = new RamseteCommand(trajectory, p_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds, leftController, rightController,
            p_Drive::tankDriveVolts, p_Drive);

      // addCommands(new TurretSetAngleCommand(p_Turret, 70).alongWith(new InstantCommand(p_Turret::setLightOn)));
      addCommands(new TurretSetAngleCommand(p_Turret, angle).alongWith(
            new SequentialCommandGroup(new InstantCommand(p_Turret::setLightOn),
                new ShootAuton(p_Shooter, p_Hopper, 3, 80000), sixBallTraj0)));
    } 
}
