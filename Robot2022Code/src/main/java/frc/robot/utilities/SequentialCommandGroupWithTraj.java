package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public abstract class SequentialCommandGroupWithTraj extends SequentialCommandGroup {

    private List<Trajectory> listofTraj = new ArrayList<Trajectory>();

    public void  addTrajectory(Trajectory traj) {
        listofTraj.add(traj);
    };
    public List<Trajectory> getTrajectories() {
        return listofTraj;
    };
    
}
