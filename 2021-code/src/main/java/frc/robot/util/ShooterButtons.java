package frc.robot.util;
import static java.lang.Math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.subsystems.TurretSubsystem;

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

    public void ShooterButtons(){
        
    }
    
    public void Periodic(){
        getTY = angle_tY.getDouble(0.0);
        double finalAngle = getTY + angle2;
        double tan = Math.tan(Math.toRadians(finalAngle));
        double distance = finalHeight / tan;

        System.out.println("Distance " + distance + " " + tan + " " + finalAngle );
    }
}
