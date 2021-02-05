// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.Path;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;




// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RamsetePathCommand extends SequentialCommandGroup {
  /** Creates a new RamsetePathCommand. */
  public RamsetePathCommand(DriveSubsystem p_Drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PIDController leftController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);
    PIDController rightController = new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0);

    RamseteController controller = new RamseteController(DriveConstants.K_RAMSETE_B, DriveConstants.K_RAMSETE_ZETA);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
        DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);
    Trajectory trajectory = new Trajectory();
        String trajectoryJSON = "paths/Unnamed.wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        RamseteCommand trajectoryRamsete = new RamseteCommand(
            trajectory, 
            p_Drive::getPose, 
            new RamseteController(DriveConstants.K_RAMSETE_B, 
                                    DriveConstants.K_RAMSETE_ZETA), 
            new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
                                        DriveConstants.KV_VOLT_SECONDS_PER_METER,
                                    DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
            DriveConstants.K_DRIVE_KINEMATICS, 
            p_Drive::getWheelSpeeds, 
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0), 
            new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
            p_Drive::tankDriveVolts, 
            p_Drive
        );
    addCommands(
      trajectoryRamsete
    );
  }
}
