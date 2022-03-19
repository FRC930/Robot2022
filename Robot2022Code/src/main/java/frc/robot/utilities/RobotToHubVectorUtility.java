package frc.robot.utilities;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotToHubVectorUtility {
    //
    public RobotToHubVectorUtility RobotVector;
    public Point hubPosition;
    double hubMagnitude;
    public double angleToHub;

public RobotToHubVectorUtility(){
    hubPosition = new Point(8.26, 4.11);
    hubMagnitude = Math.sqrt((hubPosition.x * hubPosition.x)+(hubPosition.y * hubPosition.y));
}
public double CalculateAngle(Pose2d robotPosition){
    double dotProduct = (hubPosition.x * robotPosition.getX())+(hubPosition.y * robotPosition.getY());
    double robotMagnitude = Math.sqrt((robotPosition.getX() * robotPosition.getX())+(robotPosition.getY() * robotPosition.getY()));
    double hubMagnitude = Math.sqrt((hubPosition.x * hubPosition.x)+(hubPosition.y * hubPosition.y));
    angleToHub = Math.acos(dotProduct/(hubMagnitude*robotMagnitude));
    return angleToHub;
}


}
