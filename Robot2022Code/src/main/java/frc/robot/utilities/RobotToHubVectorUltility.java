package frc.robot.utilities;

import org.opencv.core.Point;

public class RobotToHubVectorUltility {
    //
    public RobotToHubVectorUltility RobotVector;
    public Point hubPosition;
    double hubMagnitude;
    public double angleToHub;

public RobotToHubVectorUltility(){
    hubPosition = new Point(8.26, 4.11);
    hubMagnitude = Math.sqrt((hubPosition.x * hubPosition.x)+(hubPosition.y * hubPosition.y));
}
public double CalculateAngle(Point robotPosition){
    double dotProduct = (hubPosition.x * robotPosition.x)+(hubPosition.y * robotPosition.y);
    double robotMagnitude = Math.sqrt((robotPosition.x * robotPosition.x)+(robotPosition.y * robotPosition.y));
    angleToHub = Math.acos(dotProduct/(hubMagnitude*robotMagnitude));
    return angleToHub;
}


}
