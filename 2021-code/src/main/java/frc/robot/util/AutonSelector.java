// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonSelectorConstant;
import frc.robot.commands.Trajectories.GalBlueAAuton;
import frc.robot.commands.Trajectories.GalRedAAuton;
import frc.robot.commands.Trajectories.GalRedBAuton;

/** Add your docs here. */
public class AutonSelector
{
    private DriveSubsystem m_Drive;
    private IntakePIDSubsystem m_Intake;
    private GalBlueAAuton m_BlueA;
    private GalRedBAuton m_RedB;
    private GalRedAAuton m_RedA;

    NetworkTable table;
    private NetworkTableEntry angle_tX;
    private double getTX;

    public AutonSelector(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake)
    {
        m_Drive = p_Drive;
        m_Intake = p_Intake;

        table = NetworkTableInstance.getDefault().getTable("limelight");
        angle_tX = table.getEntry("tx");
        getTX = angle_tX.getDouble(0.0);

        m_BlueA = new GalBlueAAuton(m_Drive, m_Intake);
        m_RedA = new GalRedAAuton(m_Drive, m_Intake);
        m_RedB = new GalRedBAuton(p_Drive, p_Intake);
    }

    public Command getAutonCommand()
    {
        if (getTX > AutonSelectorConstant.BLUE_A)
        {
            System.out.println("Blue A");
            return m_BlueA;
        }
        else if (getTX > AutonSelectorConstant.BLUE_B)
        {
            System.out.println("Blue B");
            return m_BlueA; //change to BlueB
        }
        else if (getTX > AutonSelectorConstant.RED_B)
        {
            System.out.println("Red B");
            return m_RedB;
        }
        System.out.println("Red A");
        return m_RedA;
    }

    public void Periodic()
    {
        System.out.println(getTX);
    }
}
