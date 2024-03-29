/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

@SuppressWarnings(
{ "all" })
public class ShooterSubsystem extends SubsystemBase
{
    private TalonSRXConfiguration wheelMasterConfig = new TalonSRXConfiguration();
    private TalonSRXConfiguration wheelFollowerConfig = new TalonSRXConfiguration();
    private TalonSRX m_wheelMotor;
    private TalonSRX m_wheelMotor2;
    private TalonSRX m_wheelMotor3;
    private double m_targetVelocity;

    /**
     * Creates a new Shooter.
     */

    public ShooterSubsystem()
    {
        conifgSRX();
        m_wheelMotor = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL);
        m_wheelMotor2 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL2);
        m_wheelMotor3 = new TalonSRX(Constants.CanId.MOTOR_CONTROLLER_SHOOTERWHEEL3);

        m_wheelMotor.configAllSettings(wheelMasterConfig);
        m_wheelMotor2.configAllSettings(wheelFollowerConfig);
        m_wheelMotor3.configAllSettings(wheelFollowerConfig);
        m_wheelMotor2.follow(m_wheelMotor);
        m_wheelMotor3.follow(m_wheelMotor);

        m_wheelMotor.setInverted(true);
        m_wheelMotor2.setInverted(true);
        m_wheelMotor3.setInverted(true);

        m_wheelMotor.setNeutralMode(NeutralMode.Coast);
        m_wheelMotor2.setNeutralMode(NeutralMode.Coast);
        m_wheelMotor3.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic()
    {
    }

    public double getVelocity()
    {
        return m_wheelMotor.getSelectedSensorVelocity();
    }

    public double getTargetVelocity()
    {
        return m_targetVelocity;
    }

    public void turnOff()
    {
        m_targetVelocity = 0;
        setPower(0);
    }

    public void setPower(double power)
    {
        m_wheelMotor.set(ControlMode.PercentOutput, power);
    }

    public void setVelocity(double velocity)
    {
        m_targetVelocity = velocity;
        m_wheelMotor.set(ControlMode.Velocity, velocity);
    }

    public boolean getMotorSpeedStatus()
    {
        return false;
    }

    private void conifgSRX()
    {
        wheelMasterConfig.slot0.kP = ShooterConstants.WHEEL_P;
        wheelMasterConfig.slot0.kI = ShooterConstants.WHEEL_I;
        wheelMasterConfig.slot0.kD = ShooterConstants.WHEEL_D;
        wheelMasterConfig.slot0.kF = ShooterConstants.WHEEL_F;
        wheelMasterConfig.openloopRamp = 1;
        wheelFollowerConfig.openloopRamp = wheelMasterConfig.openloopRamp;
    }
}
