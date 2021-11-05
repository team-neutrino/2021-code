
package frc.robot.subsystems;

import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;

public class TroubleshootingSubsystem extends SubsystemBase
{
    private ShooterSubsystem m_Shooter;
    private DriveSubsystem m_Drive;
    private IntakePIDSubsystem m_Intake;
    private ClimberSubsystem m_Climber;

    private ShuffleboardTab m_troubleshooting_tab;
    private NetworkTableEntry m_a_shooter_speed;
    private NetworkTableEntry m_y_shooter_speed;
    private double m_speed_a;
    private double m_speed_y;
    private NetworkTableEntry m_shooter_velocity_two;
    private NetworkTableEntry m_navx_yaw;
    private NetworkTableEntry m_left_encoder;
    private NetworkTableEntry m_right_encoder;
    private NetworkTableEntry m_arm_angle;
    private NetworkTableEntry m_climber_height;
    private NetworkTableEntry m_just_shoot_angle;
    private double m_angle;
    public static NetworkTableEntry m_drive_distance;

    public TroubleshootingSubsystem(ShooterSubsystem p_Shooter, DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake,
            ClimberSubsystem p_Climber)
    {
        m_Shooter = p_Shooter;
        m_Drive = p_Drive;
        m_Intake = p_Intake;
        m_Climber = p_Climber;

        m_troubleshooting_tab = Shuffleboard.getTab("Troubleshooting Tab");
        m_shooter_velocity_two = m_troubleshooting_tab.add("Shooter Velocity Two", 0).withPosition(2, 0).withSize(2,
            2).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120000)).getEntry();
        m_a_shooter_speed = m_troubleshooting_tab.add("A Shooter Speed", 0).withPosition(2, 2).withSize(2,
            1).getEntry();
        m_y_shooter_speed = m_troubleshooting_tab.add("Y Shooter Speed", 0).withPosition(4, 2).withSize(2,
            1).getEntry();  
        m_climber_height = m_troubleshooting_tab.add("Climber Height", 0).withPosition(0, 0).withSize(2, 2).getEntry();

        m_navx_yaw = m_troubleshooting_tab.add("NavX Yaw", 0).withPosition(1, 3).withSize(1, 1).withProperties(
            Map.of("min", -180, "max", 180)).getEntry();
        m_left_encoder = m_troubleshooting_tab.add("Left encoder", 0).withPosition(0, 2).withSize(1, 1).getEntry();
        m_right_encoder = m_troubleshooting_tab.add("Right encoder", 0).withPosition(1, 2).withSize(1, 1).getEntry();
        m_arm_angle = m_troubleshooting_tab.add("Arm angle", 0).withPosition(0, 3).withSize(1, 1).getEntry();

        m_drive_distance = m_troubleshooting_tab.add("Drive distance", 0).withPosition(4, 4).withSize(2, 2).getEntry();
        m_just_shoot_angle = m_troubleshooting_tab.add("Just Shoot Angle", 70).withPosition(4, 4).withSize(2,
            1).getEntry();
        m_angle = m_just_shoot_angle.getNumber(70).doubleValue();
    }

    @Override
    public void periodic()
    {
        m_navx_yaw.setDouble(m_Drive.getNavxYaw());
        m_left_encoder.setDouble(m_Drive.getLeftEncoderPosition());
        m_right_encoder.setDouble(m_Drive.getRightEncoderPosition());
        m_arm_angle.setDouble(m_Intake.getMeasurement());
        m_climber_height.setDouble(m_Climber.getHeight());
        m_shooter_velocity_two.setDouble(m_Shooter.getVelocity());
        m_speed_a = m_a_shooter_speed.getNumber(0).doubleValue();
        m_speed_y = m_y_shooter_speed.getNumber(0).doubleValue();
    }

    public double getAutonAngle()
    {
        return m_angle;
    }

    public double getVelocity()
    {
        System.out.println(m_speed_a);
        return m_speed_a;
    }

    public double getVelocityY()
    {
        System.out.println(m_speed_y);
        return m_speed_y;
    }
}
