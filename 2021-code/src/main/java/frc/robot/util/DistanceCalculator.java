
package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;

public class DistanceCalculator
{
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private NetworkTableEntry angle_tY = table.getEntry("ty");
    private final double angle2 = 23;
    private final double limeLightHeight = 38;
    private final double powerPortHeight = 89.75;
    private final double finalHeight = powerPortHeight - limeLightHeight;
    private double getTY;
    private HoodSubsystem m_hood;

    public DistanceCalculator(HoodSubsystem p_hood)
    {
        m_hood = p_hood;
    }

    public double getDistance()
    {
        getTY = angle_tY.getDouble(0.0);
        double finalAngle = getTY + angle2;
        double tan = Math.tan(Math.toRadians(finalAngle));
        return finalHeight / tan;
    }

    public int getShooterSpeed()
    {
        double distance = getDistance();
        if (distance < 90)
        {
            m_hood.hoodUp();
            return Constants.ShooterConstants.green;
        }
        else if (distance > 90 && distance < 150)
        {
            m_hood.hoodUp();
            return Constants.ShooterConstants.yellow;
        }
        else if (distance > 150 && distance < 210)
        {
            m_hood.hoodUp();
            return Constants.ShooterConstants.blue;
        }
        else if (distance > 210 && distance < 270)
        {
            m_hood.hoodDown();
            return Constants.ShooterConstants.red;
        }
        return 0;
    }
}
