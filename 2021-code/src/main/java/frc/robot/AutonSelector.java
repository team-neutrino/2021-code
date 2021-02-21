// AutonSelector
// Class to select which autonomous command to run.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.commands.Trajectories.*;

public class AutonSelector
{
    // list of SequentialCommandGroup classes
    private SixBallAuton m_SixBallAuton;
    private EightBallAuton m_EightBallAuton;
    private BounceAuton m_BounceAuton;

    public AutonSelector( ShooterSubsystem p_Shooter,
        HopperSubsystem p_Hopper,
        IntakePIDSubsystem p_Intake,
        DriveSubsystem p_Drive,
        TurretSubsystem p_Turret )
    {
        m_SixBallAuton = new SixBallAuton(
            p_Shooter, p_Hopper, p_Intake, p_Drive, p_Turret);
        m_EightBallAuton = new EightBallAuton(
            p_Shooter, p_Hopper, p_Intake, p_Drive, p_Turret);
        m_BounceAuton = new BounceAuton(p_Drive);
    }

    public Command GetAuton()
    {
        return m_BounceAuton;
    }
}
