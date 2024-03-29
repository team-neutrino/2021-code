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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Trajectories.SixBallTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DumpAuton extends SequentialCommandGroup
{
    /**
     * Creates a new DumpAuton.
     */
    public DumpAuton(ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper, IntakePIDSubsystem p_Intake,
            DriveSubsystem p_Drive, TurretSubsystem p_Turret)
    {
        Trajectory trajectory = SixBallTrajectory.sixBall0;

        PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
        PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);

        RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
            DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

        // Six ball auton trajectory
        RamseteCommand sixBallTraj0 = new RamseteCommand(trajectory, p_Drive::getPose, controller, feedforward,
            DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds, leftController, rightController,
            p_Drive::tankDriveVolts, p_Drive);

        addCommands(
            // TurretSetAngleCommand coexists with the default TurretAimCommand
            // as a ParallelCommandGroup
            new InstantCommand(() -> p_Turret.setAngle(-90)).alongWith(new SequentialCommandGroup(new WaitCommand(1),
                new InstantCommand(p_Intake::setArmDown), new ShootAuton(p_Shooter, p_Hopper, 10, 80000),
                new RunCommand(() -> p_Drive.tankDrive(0.25, 0.25), p_Drive))));
    }
}
