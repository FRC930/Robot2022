//----- IMPORTS -----\\

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//----- CLASS -----\\
/**
 * <h3>LEDSubsystem</h3>
 * 
 * Manages the LEDs.
 */
public class LEDSubsystem extends SubsystemBase {

      //----- CONSTANTS ----\\

      private final int m_BUFFER_LENGTH = 299;

      private final AddressableLED m_leds;
      private final AddressableLEDBuffer m_buffer;  // Creates a new buffer object

      //----- CONSTRUCTOR -----\\
      /**
       * <h3>LEDSubsystem</h3>
       * 
       * Manages the LEDs.
       * 
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port) {

            // TODO maybe pass in buffer length
            m_leds = new AddressableLED(port); // initialization of the AdressableLED
            m_leds.setLength(m_BUFFER_LENGTH); // Sets the LED Strip length once
            m_buffer = new AddressableLEDBuffer(m_BUFFER_LENGTH);

            // TODO we start a starting color
            setBuffer(m_buffer);

            //TODO:SEE IF WE NEED THIS LINE
            m_leds.start();
      }

      //----- METHODS -----\\

      /**
       * <h3>getBufferLength</h3>
       * 
       * Returns the buffer length.
       * 
       * @return BUFFER_LENGTH
       */
      public int getBufferLength() {
            return m_BUFFER_LENGTH;
      } // End of getBufferLength()

      /**
       * <h3>getBuffer</h3>
       * 
       * Returns the LED buffer.
       * 
       * @return buffer
       */
      public AddressableLEDBuffer getBuffer() {
            return m_buffer;
      }

      /**
       * <h3>setBuffer</h3>
       * 
       * Sets the LEDs to a new buffer.
       * 
       * @param buffer  - New LED buffer
       */
      public void setBuffer(AddressableLEDBuffer buffer) {
            m_leds.setData(buffer);
      }

}
