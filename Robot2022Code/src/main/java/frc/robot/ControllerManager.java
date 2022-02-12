package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.triggers.AxisTrigger;

/**
* <h3>ControllerManager</h3>
*
* Creates the buttons so we can use them in robot container.
*/
public class ControllerManager{

    //----- CONTROLLER -----\\

    // Controller Object
    private XboxController controller;
    
    // Triggers
    private AxisTrigger leftTrigger;
    private AxisTrigger rightTrigger;

    // Left Joystick
    private AxisTrigger leftXJoystick;
    private AxisTrigger leftYJoystick;

    // Right Joystick
    private AxisTrigger rightXJoystick;
    private AxisTrigger rightYJoystick;
    
    // Bumpers
    private JoystickButton leftBumper;
    private JoystickButton rightBumper;
    
    // Letter Buttons
    private JoystickButton yButton;
    private JoystickButton aButton;
    private JoystickButton xButton;
    private JoystickButton bButton;

    // Back and Start
    private JoystickButton backButton;
    private JoystickButton startButton;

    //Joystick buttons
    private JoystickButton leftStickButton;
    private JoystickButton rightStickButton;
    

    //----- XBOX CONTROLLER IDS -----\\

    // Left Joystick
    private final int XB_AXIS_LEFT_X = 0;
    private final int XB_AXIS_LEFT_Y = 1;
    // Triggers
    private final int XB_AXIS_LT = 2;
    private final int XB_AXIS_RT = 3;
    // Right Joystick
    private final int XB_AXIS_RIGHT_X = 4;
    private final int XB_AXIS_RIGHT_Y = 5;

    // Buttons
    private final int XB_A = 1;
    private final int XB_B = 2;
    private final int XB_X = 3;
    private final int XB_Y = 4;

    // Bumpers
    private final int XB_LB = 5;
    private final int XB_RB = 6;

    // Back and Start
    private final int XB_BACK = 7;
    private final int XB_START = 8;

    // Joystick Buttons
    private final int XB_LEFTSTICK_BUTTON = 9;
    private final int XB_RIGHTSTICK_BUTTON = 10;

    /**
     * <h3>ControllerManager</h3>
     * 
     * Creates the buttons so we can use them in robot container.
     * 
     * @param port
     */
    public ControllerManager(final int port){
        controller = new XboxController(port);

        // Trigger
        leftTrigger = new AxisTrigger(controller, XB_AXIS_LT);
        rightTrigger = new AxisTrigger(controller, XB_AXIS_RT);

        // Joysticks
        leftXJoystick = new AxisTrigger(controller, XB_AXIS_LEFT_X);
        leftYJoystick = new AxisTrigger(controller, XB_AXIS_LEFT_Y);
        rightXJoystick = new AxisTrigger(controller, XB_AXIS_RIGHT_X);
        rightYJoystick = new AxisTrigger(controller, XB_AXIS_RIGHT_Y);
        
        // Bumpers
        leftBumper = new JoystickButton(controller, XB_LB);
        rightBumper = new JoystickButton(controller, XB_RB);

        // Letter Buttons
        yButton = new JoystickButton(controller, XB_Y);
        aButton = new JoystickButton(controller, XB_A);
        xButton = new JoystickButton(controller, XB_X);
        bButton = new JoystickButton(controller, XB_B);

        // Back and Start Buttons
        backButton = new JoystickButton(controller, XB_BACK);
        startButton = new JoystickButton(controller, XB_START);
        
        // Joystick Buttons
        leftStickButton = new JoystickButton(controller, XB_LEFTSTICK_BUTTON);
        rightStickButton = new JoystickButton(controller, XB_RIGHTSTICK_BUTTON);
    }

    //----- METHOD(S) -----\\

    /**
     * <h3>getLeftTrigger</h3>
     * 
     * Returns the left trigger
     * 
     * @return leftTrigger axis
     */
    public AxisTrigger getLeftTrigger(){
        return leftTrigger;
    }
    /**
     * <h3>getRightTrigger</h3>
     * 
     * Returns the Right Trigger
     * 
     * @return rightTrigger axis
     */
    public AxisTrigger getRightTrigger(){
        return rightTrigger;
    }
    /**
     * <h3>getLeftYJoystick</h3>
     * 
     * Returns the Y-axis of the Left Joystick
     * 
     * @return leftYJoystick axis
     */
    public AxisTrigger getLeftYJoystick(){
        return leftYJoystick;
    }
    /**
     * <h3>getLeftXJoystick</h3>
     * 
     * Returns the X-axis of the Left Joystick
     * 
     * @return leftXJoystick axis
     */
    public AxisTrigger getLeftXJoystick(){
        return leftXJoystick;
    }
    /**
     * <h3>getRightJoystick</h3>
     * 
     * Returns the Y-axis of the Right Joystick
     * 
     * @return RightXJoystick axis
     */
    public AxisTrigger getRightXJoystick(){
        return rightXJoystick;
    }
    /**
     * <h3>getRightYJoystick</h3>
     * 
     * Returns the Y-axis of the Right Joystick
     * 
     * @return RightYJoystick axis
     */
    public AxisTrigger getRightYJoystick(){
        return rightYJoystick;
    }


    /**
     * <h3>getLeftBumper</h3>
     * 
     * Returns the left Bumper
     * 
     * @return leftBumper
     */
    public JoystickButton getLeftBumper(){
        return leftBumper;
    }
    /**
     * <h3>getRightBumper</h3>
     * 
     * Returns the Right Bumper
     * 
     * @return rightBumper
     */
    public JoystickButton getRightBumper(){
        return rightBumper;
    }
    /**
     * <h3>getYButton</h3>
     * 
     * Returns the Y Button
     * 
     * @return ybutton
     */
    public JoystickButton getYButton(){
        return yButton;
    }
    /**
     * <h3>getAButton</h3>
     * 
     * Returns the A Button
     * 
     * @return abutton
     */
    public JoystickButton getAButton(){
        return aButton;
    }
    /**
     * <h3>getXButton</h3>
     * 
     * Returns the X Button
     * 
     * @return xbutton
     */
    public JoystickButton getXButton(){
        return xButton;
    }
    /**
     * <h3>getBButton</h3>
     * 
     * Returns the B Button
     * 
     * @return bbutton
     */
    public JoystickButton getBButton(){
        return bButton;
    }
    /**
     * <h3>getBackButton</h3>
     * 
     * Returns the back button
     * 
     * @return backButton
     */
    public JoystickButton getBackButton(){
        return backButton;
    }
    /**
     * <h3>getLeftStickButton</h3>
     * 
     * Returns the left joystick button
     * 
     * @return leftStickButton
     */
    public JoystickButton getLeftStickButton(){
        return leftStickButton;
    }
    /**
     * <h3>getRightStickButton</h3>
     * 
     * Returns the Right joystick button
     * 
     * @return rightStickButton
     */
    public JoystickButton getRightStickButton(){
        return rightStickButton;
    }
    /**
     * <h3>getStartButton</h3>
     * 
     * Returns the Start Button
     * 
     * @return startButton
     */
    public JoystickButton getStartButton(){
        return startButton;
    }

    /**
     * <h3>getController</h3>
     * 
     * Returns the controller
     * 
     * @return controller
     */
    public XboxController getController(){
        return controller;
    }
}
