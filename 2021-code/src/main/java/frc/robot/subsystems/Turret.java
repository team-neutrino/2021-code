
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;

public class Turret extends SubsystemBase
{
    /** Creates a new Turret. */

    private TalonSRX m_turretMotor;

    private int angle;

    public Turret()
    {
        m_turretMotor = new TalonSRX(CanId.MOTOR_CONTROLLER_TURRET);
    }

    // public void setAngle(int newAngle)
    // {
    //   double difference = angle - newAngle;
    //   if(difference < 5) {
    //     stop();
    //   }
    //   m_turretMotor.set(ControlMode.PercentOutput, 0.2);
    // }

    public void rotateLeft()
    {
        m_turretMotor.set(ControlMode.PercentOutput, 0.2);
    }

    public void rotateRight()
    {
        m_turretMotor.set(ControlMode.PercentOutput, -0.2);
    }

    public void stop()
    {
        m_turretMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public int getTurretAngle()
    {
      return m_turretMotor.getSelectedSensorPosition();
    }



    @Override
    public void periodic()
    {
        angle = getTurretAngle();
        // This method will be called once per scheduler run
    }
}
