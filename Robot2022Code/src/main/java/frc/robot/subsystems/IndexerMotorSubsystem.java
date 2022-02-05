package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerMotorSubsystem extends SubsystemBase {

    private final WPI_TalonSRX indexerMotor;

    public IndexerMotorSubsystem(int id){
        indexerMotor = new WPI_TalonSRX(id);
        indexerMotor.stopMotor();
    }

    /**
     * <h3>setMotorSpeed</h3>
     * 
     * Sets the indexer motor speed.
     * @param speed the speed to set the motor
     */
    public void setMotorSpeed(double speed) {
        indexerMotor.set(speed);
    }
}
