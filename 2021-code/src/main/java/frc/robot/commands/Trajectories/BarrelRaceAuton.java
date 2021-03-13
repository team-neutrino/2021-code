package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Trajectories.BarrelRaceTrajectory;
import frc.robot.Trajectories.BounceTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class BarrelRaceAuton extends SequentialCommandGroup {
    private DriveSubsystem m_Drive;

    public BarrelRaceAuton(DriveSubsystem p_Drive) {
        m_Drive = p_Drive;
        Trajectory m_barrelRace0 = BarrelRaceTrajectory.barrelRace0;
        Trajectory m_barrelRace1 = BarrelRaceTrajectory.barrelRace1;
        Trajectory m_barrelRace2 = BarrelRaceTrajectory.barrelRace2;
        
        RamseteCommand barrelRace0 = new RamseteCommand(m_barrelRace0, p_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

            p_Drive::tankDriveVolts, p_Drive);

            RamseteCommand barrelRace1 = new RamseteCommand(m_barrelRace1, p_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

            p_Drive::tankDriveVolts, p_Drive);

            RamseteCommand barrelRace2 = new RamseteCommand(m_barrelRace2, p_Drive::getPose,
            new RamseteController(Constants.DriveConstants.K_RAMSETE_B, Constants.DriveConstants.K_RAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS,
                Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            Constants.DriveConstants.K_DRIVE_KINEMATICS, p_Drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),
            new PIDController(Constants.DriveConstants.KP_DRIVE_VEL, 0, 0),

            p_Drive::tankDriveVolts, p_Drive);

            addCommands(
                new SequentialCommandGroup(
                    barrelRace0,
                    barrelRace1,
                    barrelRace2,
                    new InstantCommand(() -> m_Drive.tankDriveVolts(0, 0))));
    }
}
