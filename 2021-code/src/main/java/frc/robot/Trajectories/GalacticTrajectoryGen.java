package frc.robot.Trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

//trajectory generation (numbers to be adjusted)
public class GalacticTrajectoryGen {
    public static Trajectory GalacticTrajectory;

    public GalacticTrajectoryGen() {
        
      // 2018 cross scale auto waypoints.
      var sideStart = new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(5),
          Rotation2d.fromDegrees(0));
      var crossScale = new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(5),
          Rotation2d.fromDegrees(-90));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(5), Units.feetToMeters(5)));
      
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(5), Units.feetToMeters(6.6));
      config.setReversed(false);
  
        var trajectory = TrajectoryGenerator.generateTrajectory(
            sideStart,
            interiorWaypoints,
            crossScale,
            config);
        GalacticTrajectory = trajectory;
    }
  }