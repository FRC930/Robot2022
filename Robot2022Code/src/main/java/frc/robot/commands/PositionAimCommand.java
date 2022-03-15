package frc.robot.commands;

import javax.swing.text.Position;

import org.opencv.core.Point;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.RobotToHubVectorUltility;

public class PositionAimCommand {
    DriveSubsystem dSubsystem;
    RobotToHubVectorUltility robotToHubVectorUltility;
    
    

    // vector angle minus the current heading of our robot
    // @Override
    // public void ex`ecute() {
    //     dSubsystem.drive(1, robotToHubVectorUltility.CalculateAngle(dSubsystem.getOdometry().getPoseMeters().));
        
    // }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    
    
}
