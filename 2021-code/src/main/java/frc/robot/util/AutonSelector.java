// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.RamseteGenCommand;
import frc.robot.commands.Trajectories.BounceAuton;
import frc.robot.commands.Trajectories.GalBlueA;
import frc.robot.commands.Trajectories.GalRedA;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;

/** Add your docs here. */
public class AutonSelector
{
    private AnalogInput input = new AnalogInput(1);
    private AnalogPotentiometer analogPot = new AnalogPotentiometer(input, 511.77);
    private RamseteGenCommand m_RamseteGen;
    private DriveSubsystem m_Drive;
    private IntakePIDSubsystem m_Intake; 
    private GalBlueA m_BlueA;
    private GalRedA m_RedA;
    public AutonSelector(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake)
    {
        m_Drive = p_Drive;
        m_Intake = p_Intake;
        m_BlueA = new GalBlueA(m_Drive, m_Intake);
        m_RedA = new GalRedA(m_Drive, m_Intake);
        System.out.println(analogPot.get());  
    }

    public Command getAutonCommand() {
        double dist = analogPot.get();
        
        if (0 < dist && dist < 50)
        {
            return m_RedA;
        }
        else
        {
            return m_BlueA;
        }   
    }

    public void Periodic()
    {
        System.out.println(analogPot.get());
    }

}
