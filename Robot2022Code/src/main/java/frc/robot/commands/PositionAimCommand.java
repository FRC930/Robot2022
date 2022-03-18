package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.RobotToHubVectorUltility;
import frc.robot.utilities.ShuffleboardUtility;

public class PositionAimCommand extends CommandBase {
    DriveSubsystem dSubsystem;
    RobotToHubVectorUltility robotToHubVectorUltility;
    final double ANGULAR_P = 0.4;
    final double ANGULAR_D = 0.01;
    double rotationSpeed;
    PIDController turnController;
    double targetHeading;

    public PositionAimCommand(DriveSubsystem driveSubsystem){
        dSubsystem = driveSubsystem;
        robotToHubVectorUltility = new RobotToHubVectorUltility();
        turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

        addRequirements(dSubsystem);
    }
    // vector angle minus the current heading of our robot
    @Override
    public void initialize(){
        double angleDifference = robotToHubVectorUltility.CalculateAngle(dSubsystem.getOdometry().getPoseMeters());
        if(GyroUtility.getInstance().getGyro().getYaw() > 0){
            targetHeading = GyroUtility.getInstance().getGyro().getYaw() + angleDifference;
        }
        else{
            targetHeading = GyroUtility.getInstance().getGyro().getYaw() - angleDifference;
        }
        SmartDashboard.putNumber("targetHeading", targetHeading);
        SmartDashboard.putNumber("angleDifference", angleDifference);
        SmartDashboard.putNumber("poseY", dSubsystem.getOdometry().getPoseMeters().getY());
        SmartDashboard.putNumber("poseX", dSubsystem.getOdometry().getPoseMeters().getX());
    }


    @Override
    public void execute() {
        rotationSpeed = turnController.calculate(GyroUtility.getInstance().getGyro().getYaw(), targetHeading);
        dSubsystem.drive(0, rotationSpeed);
        SmartDashboard.putNumber("RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber("getYaw", GyroUtility.getInstance().getGyro().getYaw());
        SmartDashboard.putNumber("target heading", targetHeading);
    }

    @Override
    public boolean isFinished() {
        if(rotationSpeed == 0){
            return true;
        }
        else{
            return false;
        }
    }
    
    
}
