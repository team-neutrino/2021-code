/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Implement in replacement of SixBallAuto, in the case that SixBallAuto is ineffective due to match specifics.
 */
public class ThreeAuton extends CommandBase
{
    private ShooterSubsystem m_Shooter;
    private HopperSubsystem m_Hopper;
    private DriveSubsystem m_Drive;
    private TurretSubsystem m_Turret;
    private Timer m_Timer = new Timer();
    private double m_Angle;

    /**
     * Creates a new ShootAuton.
     */
    public ThreeAuton(ShooterSubsystem p_Shooter, HopperSubsystem p_Hopper, DriveSubsystem p_Drive,
            TurretSubsystem p_Turret, double p_Angle)
    {
        addRequirements(p_Shooter, p_Hopper, p_Drive);
        m_Shooter = p_Shooter;
        m_Hopper = p_Hopper;
        m_Drive = p_Drive;
        m_Turret = p_Turret;
        m_Angle = p_Angle;
    }

    /**
     * Called when the command is initially scheduled.
     */
    @Override
    public void initialize()
    {
        m_Timer.start();
        m_Turret.setpointSetAngle(70);
        m_Turret.setLightOn();
        m_Shooter.setVelocity(78000);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute()
    {
        if (m_Timer.get() > 2.5)
        {
            m_Hopper.towerShoot();
        }
        if (m_Timer.get() >= 5)
        {
            m_Shooter.setPower(0);
            m_Drive.tankDrive(-0.25, -0.25);
        }

    }

    /**
     * Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted)
    {
        m_Turret.setLightOn();
        m_Hopper.stop();
    }

    /**
     * @return true when command should
     */
    @Override
    public boolean isFinished()
    {
        return m_Timer.get() >= 6;
    }
}
