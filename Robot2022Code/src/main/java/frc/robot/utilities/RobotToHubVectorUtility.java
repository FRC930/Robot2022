package frc.robot.utilities;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotToHubVectorUtility {
    //
    public RobotToHubVectorUtility RobotVector;
    public Point hubPosition;
    public double angleToHub;

    public RobotToHubVectorUtility(){
        hubPosition = new Point(8.26, 4.11);
    }
    public double CalculateAngle(Pose2d robotPosition){
        double radian = Math.atan2(hubPosition.y - robotPosition.getY(), hubPosition.x - robotPosition.getX());
        double angleToHub = (radian * (180 / Math.PI)) + 180; 
        SmartDashboard.putNumber("calculatedRadian", radian);
        return new Rotation2d(radian + Math.PI).getDegrees();

    }
}
