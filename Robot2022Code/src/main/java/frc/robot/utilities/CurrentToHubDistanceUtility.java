package frc.robot.utilities;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class CurrentToHubDistanceUtility {
    public Point hubPosition;
    double distanceToHub;

    public CurrentToHubDistanceUtility(){
        hubPosition = new Point(8.26, 4.11);
    }

    public double getDistanceToHub(Pose2d robotPosition){
        distanceToHub = Units.metersToFeet(Math.sqrt(((robotPosition.getX() - hubPosition.x) * (robotPosition.getX() - hubPosition.x)) + ((robotPosition.getY() - hubPosition.y) * (robotPosition.getY() - hubPosition.y))));
        return distanceToHub;
    }
}