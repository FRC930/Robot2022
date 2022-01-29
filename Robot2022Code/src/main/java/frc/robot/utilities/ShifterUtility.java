package frc.robot.utilities;

public class ShifterUtility {
    // False is low gear, true is high gear
    private static boolean m_shifterState = false;

    public static void setShifterState(boolean shifterState) {
        m_shifterState = shifterState;
    }

    public static boolean getShifterState() {
        return m_shifterState;
    }
}
