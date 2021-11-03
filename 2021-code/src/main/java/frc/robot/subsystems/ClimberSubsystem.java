/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private TalonSRX m_ClimbElevator = new TalonSRX(CanId.MOTOR_CONTROLLER_CLIMBER);
    private CANSparkMax m_ClimbWinch = new CANSparkMax(CanId.MOTOR_CONTROLLER_CLIMBERWINCH, MotorType.kBrushless);
    private static boolean elevatorPressed = false;
    private static boolean backButton = false;
    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem()
    {
        m_ClimbElevator.setNeutralMode(NeutralMode.Brake);
        elevatorPressed = false;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        if (getHeight() < 0)
        {
            m_ClimbElevator.setSelectedSensorPosition(0);
        }
    }

    public void elevatorUp()
    {
        System.out.println("ELEVATOR UP - back button: " + backButton + " elevator pressed:" + elevatorPressed);
        if(backButton == false)
        {
            m_ClimbElevator.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_MOTOR_POWER_UP);
        }
        elevatorPressed = true;
    }

    public void elevatorDown()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, -ClimberConstants.CLIMBER_MOTOR_POWER_DOWN);
    }

    public void winchClimb()
    {
        System.out.println("WINCH UP - back button: " + backButton + " elevator pressed:" + elevatorPressed);
        if (elevatorPressed && backButton)
        {
            m_ClimbWinch.set(ClimberConstants.CLIMBER_MOTOR_WINCHPOWER);
        }
        backButton = false;
    }

    public void backButtonPressed()
    {
        backButton = true;

    }

    public void backButtonUnpressed()
    {
        backButton = false;
    }

    public void elevatorStop()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, 0);
    }

    public void winchStop()
    {
        m_ClimbWinch.set(0);
    }

    public void winchReverse()
    {
        if (elevatorPressed)
        {
            m_ClimbWinch.set(-ClimberConstants.CLIMBER_MOTOR_WINCHPOWER);
        }
    }

    public double getHeight()
    {
        return m_ClimbElevator.getSelectedSensorPosition();
    }
}
