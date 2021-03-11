package frc.robot.util;

import static java.lang.Math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShooterSetSpeedCommand;
import frc.robot.subsystems.ShooterSubsystem;
import static edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.*;

public class ShooterButtons {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private int green = 62500;
    private int blue = 65000;
    private int red = 69000;
    private int yellow = 61000;
    private NetworkTableEntry angle_tY = table.getEntry("ty");
    private final double angle2 = 23;
    private final double limeLightHeight = 38;
    private final double powerPortHeight = 89.75;
    private final double finalHeight = powerPortHeight - limeLightHeight;
    private double getTY;
    private XboxController m_OperatorController = new XboxController(ControllerPorts.XBOX_CONTROLLER_PORT);

    private JoystickButton m_A = new JoystickButton(m_OperatorController, Button.kA.value);
    private JoystickButton m_B = new JoystickButton(m_OperatorController, Button.kB.value);
    private JoystickButton m_X = new JoystickButton(m_OperatorController, Button.kX.value);
    private JoystickButton m_Y = new JoystickButton(m_OperatorController, Button.kY.value);
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem();



    public void ShooterButtons(){
        
    }
    
    public void Periodic(){
        getTY = angle_tY.getDouble(0.0);
        double finalAngle = getTY + angle2;
        double tan = Math.tan(Math.toRadians(finalAngle));
        double distance = finalHeight / tan;
        if(distance < 90){
            m_A.whenHeld(new ShooterSetSpeedCommand(m_Shooter, green));
        }
        else if(distance > 90 && distance <150){
            m_A.whenHeld(new ShooterSetSpeedCommand(m_Shooter, yellow));
        }
        else if(distance >150 && distance <210){
            m_A.whenHeld(new ShooterSetSpeedCommand(m_Shooter, blue));
        }
        else if(distance > 210 && distance < 270){
            m_A.whenHeld(new ShooterSetSpeedCommand(m_Shooter, red));
        }


        //System.out.println("Distance " + distance + " " + tan + " " + finalAngle );
    }
}
