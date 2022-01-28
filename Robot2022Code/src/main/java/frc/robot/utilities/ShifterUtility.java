package frc.robot.utilities;

public class ShifterUtility {
    private static boolean m_shifterState = false;

    public static void setShifterState(boolean shifterState) {
        m_shifterState = shifterState;
    }

    public static boolean getShifterState() {
        return m_shifterState;
    }
}
