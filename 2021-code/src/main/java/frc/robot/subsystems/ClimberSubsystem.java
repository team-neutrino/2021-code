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
import frc.robot.subsystems.TurretSubsystem;

public class ClimberSubsystem extends SubsystemBase
{
    private TalonSRX m_ClimbElevator = new TalonSRX(CanId.MOTOR_CONTROLLER_CLIMBER);
    private CANSparkMax m_ClimbWinch = new CANSparkMax(CanId.MOTOR_CONTROLLER_CLIMBERWINCH, MotorType.kBrushless);
    private static boolean elevatorPressed = false;
    private TurretSubsystem m_Turret;
    /**
     * Creates a new ClimberSubsystem.
     */
    public ClimberSubsystem(TurretSubsystem p_Turret)
    {
        m_ClimbElevator.setNeutralMode(NeutralMode.Brake);
        elevatorPressed = false;
        m_Turret = p_Turret;
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
        m_Turret.setLightOff();
        if (getHeight() > ClimberConstants.CLIMBER_FULL_EXTEND)
        {
            elevatorStop();
        }
        else
        {
            m_ClimbElevator.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_MOTOR_POWER_UP);
            m_Turret.setLightOff();
        }
    }

    public void elevatorDown()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, -ClimberConstants.CLIMBER_MOTOR_POWER_DOWN);
    }

    public void winchClimb()
    {
        if (elevatorPressed)
        {
            m_ClimbWinch.set(ClimberConstants.CLIMBER_MOTOR_WINCHPOWER);
        }
    }

    public void elevatorStop()
    {
        m_ClimbElevator.set(ControlMode.PercentOutput, 0);
        m_Turret.setLightOff();
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
