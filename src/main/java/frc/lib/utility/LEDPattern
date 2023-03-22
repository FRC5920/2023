package frc.robot.utility;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDPattern {
    private final int kPatternLength = 6;
    private Color8Bit m_pattern[];
    private int m_arraySize;  /**length of LED strip */
    private int m_startIndex; /**starting index in the LED strip */
    private int m_iteration;  /**number of times array has been processed */
    private int m_arrayPos;  /**current position to start writing into the LED strip array */
    private int m_patternStart; /**where in the pattern to begin to copy from */
    private int m_patternEnd;  /**where in the pattern to stop copying */

    public enum Direction {kForward, kReverse};
    private Direction m_direction;

    /**
     * Creates an LEDPattern object that will display a pattern
     * in an LED strip
     * @param startIndex  Starting index in the LED array where
     *                    the pattern will be displayed
     * @param numLEDs Number of LEDs in the strip to walk the pattern through
     * @param color   The color of the pattern
     */
    public LEDPattern(int startIndex, int numLEDs, Color8Bit color) {
        m_startIndex = startIndex;
        m_arraySize = numLEDs;
        m_direction = Direction.kForward;
        reset();

        // Initialize the LED pattern array
        m_pattern =  new Color8Bit[kPatternLength];
        setColor(color);
    }

    /** Sets the color displayed by the pattern
     * @param color  The color the pattern should display
     */
    public void setColor(Color8Bit color) {
        m_pattern[0] = scaleIntensity(color, 1.0);
        m_pattern[1] = scaleIntensity(color, 0.75);
        m_pattern[2] = scaleIntensity(color, 0.5);
        m_pattern[3] = scaleIntensity(color, 0.25);
        m_pattern[4] = scaleIntensity(color, 0.125);
        m_pattern[5] = scaleIntensity(color, 0.06);
    }

    /** Resets the LED pattern to its initial position in the target
     *  LED strip
    */
    public void reset() {
        m_iteration = 0;
        m_arrayPos = 0;
        m_patternStart = 0;
        m_patternEnd = 0;
    }

    /** Translate an index in the LED array according to whether 
     *  the pattern is being moved forward or reverse in the array
     */
    private int translateIndex(int index) {
        int result = index;
        if (m_direction == Direction.kReverse) {
            result = (m_arraySize - 1) - index;
        }

        return result;
    }

    /** Helper function used to scale the intensity of a given Color to
     *  produce a Color8Bit
     * 
     * @param color  The color to scale
     * @param intensity  Scale factor to apply the color intensity (0.0 to 1.0)
     */
    private static Color8Bit scaleIntensity(Color8Bit color, double scaleFactor) {
        int red =  (int)(color.red * scaleFactor);
        int green = (int)(color.green *scaleFactor);
        int blue = (int)(color.blue *scaleFactor);
        return new Color8Bit(red, green, blue);
    }

    /** Display's this object's pattern on a given LED array
     * 
     * @param ledBuffer  The LED strip array to display in
     */
    public void process(AddressableLEDBuffer ledBuffer) {
        Color8Bit off = new Color8Bit(0, 0, 0);
        for (int i = m_startIndex; i < m_startIndex + m_arraySize; ++i) {
            ledBuffer.setLED(i, off);
        }
        
        // Copy the pattern into ledArray starting at the current
        // array index
        int a = m_startIndex + translateIndex(m_arrayPos);
        for (int p = m_patternStart; p <= m_patternEnd; ++p) {
            Color8Bit c = m_pattern[p];
            ledBuffer.setRGB(a, c.red, c.green, c.blue);
            a += (m_direction == Direction.kForward) ? -1 : 1;
        }

        m_iteration += 1;
        if (m_iteration > (m_arraySize + kPatternLength - 2)) {
            reset();
        }
        else {
            if (m_arrayPos < (m_arraySize - 1)) {
                m_arrayPos += 1;
            }

            if (m_iteration >= m_arraySize)
            {
                m_patternStart += 1;
            }

            if (m_patternEnd < (kPatternLength - 1)) {
                m_patternEnd += 1;
            }
        }
    }
}