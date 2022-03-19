package frc.robot.commands.autovisioncommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.GyroUtility;
import frc.robot.utilities.RobotToHubVectorUtility;
import frc.robot.utilities.ShuffleboardUtility;

public class PigeonAimCommand extends CommandBase {
    DriveSubsystem dSubsystem;
    RobotToHubVectorUtility robotToHubVectorUtility;
    final double ANGULAR_P = 0.03;
    final double ANGULAR_D = 0.0;
    double rotationSpeed;
    PIDController turnController;
    double targetHeading;
    Double robotHeading;

    public PigeonAimCommand(DriveSubsystem driveSubsystem){
        robotHeading = 0.0;
        dSubsystem = driveSubsystem;
        robotToHubVectorUtility = new RobotToHubVectorUtility();
        turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

        addRequirements(dSubsystem);
    }
    // vector angle minus the current heading of our robot
    @Override
    public void initialize(){
        targetHeading = robotToHubVectorUtility.CalculateAngle(dSubsystem.getOdometry().getPoseMeters());
        SmartDashboard.putNumber("targetHeading", targetHeading);
        SmartDashboard.putNumber("poseY", dSubsystem.getOdometry().getPoseMeters().getY());
        SmartDashboard.putNumber("poseX", dSubsystem.getOdometry().getPoseMeters().getX());
    }


    @Override
    public void execute() {
        robotHeading = dSubsystem.getOdometry().getPoseMeters().getRotation().getDegrees();
        if(Math.signum(robotHeading) == -1){
            turnController.setP(-1 * ANGULAR_P);
        }
        else{
            turnController.setP(ANGULAR_P);
        }
        
        rotationSpeed = turnController.calculate(robotHeading, targetHeading);

        var wheelSpeeds = dSubsystem.getWheelSpeeds(0, rotationSpeed);

        dSubsystem.setVoltages(
                // Calculate feedforward with the feedforward controller in drive subsystem
                dSubsystem.calculateLeftFeedforward(
                        // Use the speed to voltage method in the drive subsystem
                        wheelSpeeds.leftMetersPerSecond),
                // Same deal here, feedforward using the helper method
                dSubsystem.calculateRightFeedforward(
                        // Again, speed to voltage
                        wheelSpeeds.rightMetersPerSecond));

        SmartDashboard.putNumber("RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber("fusedHeading", robotHeading);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(robotHeading - targetHeading) < 6) {
            return true;
        }
        else{
            return false;
        }
    }
    
    
}