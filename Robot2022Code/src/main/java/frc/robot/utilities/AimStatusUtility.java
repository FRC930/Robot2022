//----- IMPORTS -----\\

package frc.robot.utilities;

//----- CLASS -----\\
/**
 * 
 */
public class AimStatusUtility {

    //----- VARIABLES -----\\

    private boolean m_aimed = false;
    
    private static AimStatusUtility m_instance = null;

    //----- CONSTRUCTOR -----\\
    /**
     * 
     */
    private AimStatusUtility() {

    }

    //----- METHODS -----\\

    /**
     * <h3>getInstance</h3>
     * 
     * @return
     */
    public static AimStatusUtility getInstance() {
        if(m_instance == null) {
            m_instance = new AimStatusUtility();
        }

        return m_instance;
    }

    /**
     * <h3>getAimStatus</h3>
     * 
     * @return
     */
    public boolean getAimStatus() {
        return m_aimed;
    }

    public void setAimStatus(boolean aimed) {
        m_aimed = aimed;
    }

}
