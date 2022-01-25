package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatapultSensorSubsystem extends SubsystemBase{
    private DigitalInput catapultSensor;

    public CatapultSensorSubsystem(int portDio){
        catapultSensor = new DigitalInput(portDio);
    }

    public DigitalInput getDigitalInput(){
        return catapultSensor;
    }

    public boolean isActivated() {
        return catapultSensor.get();
    }
}
