
package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants;

public class ShooterButtons
{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry angle_tY = table.getEntry("ty");
    private final double angle2 = 23;
    private final double limeLightHeight = 38;
    private final double powerPortHeight = 89.75;
    private final double finalHeight = powerPortHeight - limeLightHeight;
    private double getTY;

    public void ShooterButtons()
    {
    
    }

    public int getDistance(JoystickButton m_B)
    {
        getTY = angle_tY.getDouble(0.0);
        double finalAngle = getTY + angle2;
        double tan = Math.tan(Math.toRadians(finalAngle));
        double distance = finalHeight / tan;
        if (distance < 90)
        {
            return Constants.ShooterConstants.green;
        }
        else if (distance > 90 && distance < 150)
        {
            return Constants.ShooterConstants.yellow;
        }
        else if (distance > 150 && distance < 210)
        {
            return Constants.ShooterConstants.blue;
        }
        else if (distance > 210 && distance < 270)
        {
            return Constants.ShooterConstants.red;
        }
        return 0;
    }
}
