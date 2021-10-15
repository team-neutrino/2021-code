// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakePIDSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonSelectorConstant;
import frc.robot.commands.Trajectories.GalBlueAAuton;
import frc.robot.commands.Trajectories.GalRedAAuton;
import frc.robot.commands.Trajectories.GalRedBAuton;
import frc.robot.commands.Trajectories.SixBallAuton;
import frc.robot.commands.Trajectories.GalBlueBAuton;

/** Add your docs here. */
public class AutonSelector
{
    private DriveSubsystem m_Drive;
    private IntakePIDSubsystem m_Intake;
    private GalBlueAAuton m_BlueA;
    private GalRedBAuton m_RedB;
    private GalRedAAuton m_RedA;
    public GalBlueBAuton m_BlueB;
    public SixBallAuton m_Six;

    private NetworkTable table;

    public AutonSelector(DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake)
    {
        m_Drive = p_Drive;
        m_Intake = p_Intake;

        m_BlueA = new GalBlueAAuton(m_Drive, m_Intake);
        m_RedA = new GalRedAAuton(m_Drive, m_Intake);
        m_RedB = new GalRedBAuton(p_Drive, p_Intake);
        m_BlueB = new GalBlueBAuton(p_Drive, p_Intake);

        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTx()
    {
        NetworkTableEntry angle_tX = table.getEntry("tx");
        double getTX = angle_tX.getDouble(0.0);

        return getTX;
    }

    public double getTx1()
    {
        NetworkTableEntry angle_tX0 = table.getEntry("tx1");
        double getTX1 = angle_tX0.getDouble(0.0);

        return getTX1;
    }

    public String getPath()
    {
        if (getTx1() < AutonSelectorConstant.RED_A_TX1 && getTx() > AutonSelectorConstant.RED_A)
        {
            return "Red A";
        }

        if (getTx() > AutonSelectorConstant.BLUE_A)
        {
            return "Blue A";
        }
        else if (getTx() > AutonSelectorConstant.RED_B)
        {
            return "Red B";
        }
        else if (getTx() > AutonSelectorConstant.BLUE_B)
        {
            return "Blue B";
        }

        return "None";
    }

    public Command getAutonCommand()
    {
        if (getPath().equals("Blue A"))
        {
            System.out.println("Blue A");
            return m_BlueA;
        }
        else if (getPath().equals("Red B"))
        {
            System.out.println("Red B");
            return m_RedB;
        }
        else if (getPath().equals("Blue B"))
        {
            System.out.println("Blue B");
            return m_BlueB;
        }
        else if (getPath().equals("Red A"))
        {
            System.out.println("Red A");
            return m_RedA;
        }
        System.out.println("None");
        return m_RedA;
    }
}
