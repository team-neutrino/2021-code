// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;
import frc.robot.commands.RamseteGenCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutonSelector
{
    private AnalogInput input = new AnalogInput(1);
    private AnalogPotentiometer analogPot = new AnalogPotentiometer(input, 511.77);
    private final DriveSubsystem m_Drive = new DriveSubsystem();
    private RamseteGenCommand m_RamseteGen;
    public AutonSelector()
    {
        /*
         * m_SixBallAuto = new SixBallAuto(m_Shooter, m_Hopper, m_Intake, m_Drive, m_Turret); m_DumpAuton = new
         * DumpAuton(m_Shooter, m_Hopper, m_Intake, m_Drive, m_Turret); m_ThreeAuton = new ThreeAuton(m_Shooter,
         * m_Hopper, m_Drive, 10); m_EightBallAuto = new EightBallAuto(m_Shooter, m_Hopper, m_Intake, m_Drive,
         * m_Turret);
         */
        // System.out.println(analogPot.get());
        // if (140 < analogPot.get() && analogPot.get() < 160)
        // {
        //     m_RamseteGen = new RamseteGenCommand(m_Drive, Constants.PathConstants.GALACTIC_RED_A_PATH);
        // }
        // else
        // {
        //     m_RamseteGen = new RamseteGenCommand(m_Drive, Constants.PathConstants.GALACTIC_BLUE_A_PATH);

        // }
    }

    public void Periodic()
    {
        System.out.println(analogPot.get());

    }

}
