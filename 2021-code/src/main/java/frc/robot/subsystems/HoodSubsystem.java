// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase
{
    private Servo m_servo = new Servo(0);
    /** Creates a new HoodSubsystem. */
    public HoodSubsystem()
    {
    }

    public void hoodDown()
    {
        m_servo.setAngle(Constants.HoodConstants.HOOD_ANGLE_DOWN);
        System.out.println("hoodDown()");
    }

    public void hoodUp()
    {
        m_servo.setAngle(Constants.HoodConstants.HOOD_ANGLE_UP);
        System.out.println("hoodUp()");
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
