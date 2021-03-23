
package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Trajectories.GalacticABlueTrajectory;
import frc.robot.Trajectories.GalacticBRedTrajectory;

public class GalRedBAuton extends SequentialCommandGroup
{
    Trajectory m_galRedB = GalacticBRedTrajectory.galRedB;
    public GalRedBAuton(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake)
    {
        RamseteCommand redB = new RamseteCommand(m_galRedB, p_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

            p_Drive::tankDriveVolts, p_Drive);

        addCommands(new InstantCommand(p_Intake::setIntakeOn), redB,
            new InstantCommand(() -> p_Drive.tankDriveVolts(0, 0)));
    }
}