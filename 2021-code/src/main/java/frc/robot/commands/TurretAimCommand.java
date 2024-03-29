/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAimCommand extends CommandBase
{

    private TurretSubsystem m_turret;

    /**
     * Creates a new TurretAimCommand.
     */
    public TurretAimCommand(TurretSubsystem p_turret)
    {
        addRequirements(p_turret);
        m_turret = p_turret;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        m_turret.setLightOn();
        System.out.println("turretAimCommand setLightOn");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        m_turret.autoSetAngle();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        // Stops turret when command ends
        m_turret.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
