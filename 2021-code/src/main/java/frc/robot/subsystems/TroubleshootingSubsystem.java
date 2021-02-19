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
    private NetworkTableEntry m_input_shooter_Speed;
    private double m_speed;
    private NetworkTableEntry m_shooter_velocity_two;
    private NetworkTableEntry m_navx_yaw;
    private NetworkTableEntry m_navx_pitch;
    private NetworkTableEntry m_navx_roll;
    private NetworkTableEntry m_left_encoder;
    private NetworkTableEntry m_right_encoder;
    private NetworkTableEntry m_arm_angle;
    private NetworkTableEntry m_climber_height;


    public TroubleshootingSubsystem(ShooterSubsystem p_Shooter, DriveSubsystem p_Drive, IntakePIDSubsystem p_Intake, ClimberSubsystem p_Climber) {
        m_Shooter = p_Shooter;
        m_Drive = p_Drive;
        m_Intake = p_Intake;
        m_Climber = p_Climber;

        m_troubleshooting_tab = Shuffleboard.getTab("Troubleshooting Tab");
        m_shooter_velocity_two = m_troubleshooting_tab.add("Shooter Velocity Two", 0).withPosition(2, 0).withSize(2, 2).withWidget(
            BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 120000)).getEntry();
        m_input_shooter_Speed = m_troubleshooting_tab.add("Input Shooter Velocity", 0).withPosition(4,0).withSize(1,1).getEntry();
        m_arm_angle = m_troubleshooting_tab.add("Arm angle", 0).withPosition(1, 0).withSize(1, 1).getEntry();
        m_climber_height = m_troubleshooting_tab.add("Height of Climber", 0).withPosition(1, 2).withSize(1, 1).getEntry();

        m_navx_yaw = m_troubleshooting_tab.add("NavX Yaw", 0).withPosition(0, 0).withSize(1, 1).withProperties(Map.of("min", -180, "max", 180)).getEntry();
        // m_navx_pitch = m_troubleshooting_tab.add("NavX Pitch", 0).withPosition(2, 3).withSize(1, 1).withProperties(Map.of("min", -180, "max", 180)).getEntry();
        // m_navx_roll = m_troubleshooting_tab.add("NavX Roll", 0).withPosition(2, 3).withSize(1, 1).withProperties(Map.of("min", -180, "max", 180)).getEntry();
        m_left_encoder = m_troubleshooting_tab.add("Left encoder", 0).withPosition(1, 0).withSize(1, 1).getEntry();
        m_right_encoder = m_troubleshooting_tab.add("Right encoder", 0).withPosition(2, 0).withSize(1, 1).getEntry();
    }

    @Override
    public void periodic()
    {
        m_navx_yaw.setDouble(m_Drive.getNavxYaw());
        m_navx_pitch.setDouble(m_Drive.getNavxPitch());
        m_navx_roll.setDouble(m_Drive.getNavxRoll());
        m_left_encoder.setDouble(m_Drive.getLeftEncoderPosition());
        m_right_encoder.setDouble(m_Drive.getRightEncoderPosition());
        m_arm_angle.setDouble(m_Intake.getMeasurement());
        m_climber_height.setDouble(m_Climber.getHeight());
        m_shooter_velocity_two.setDouble(m_Shooter.getVelocity());
        m_speed = m_input_shooter_Speed.getNumber(0).doubleValue();
    }

    public double getVelocity()
    {
        System.out.println(m_speed);
        return m_speed;  
    }
}
