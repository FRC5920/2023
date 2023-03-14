////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5920 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/
package frc.lib.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.utility.PIDGains;
import java.util.Map;
import java.util.function.Supplier;

/** Add your docs here. */
public class WidgetsWithChangeDetection {

  public static class ChangeDetector<V> {
    private Supplier<V> m_valueSupplier;

    /** Initial return value from hasChanged() */
    private boolean m_forceInitialChange;

    /** Last value registered in hasChanged() */
    private V m_lastValue;

    /**
     * Creates an instance of the object that will return true the first time hasChanged() is
     * called.
     */
    public ChangeDetector(Supplier<V> valueSupplier) {
      this(valueSupplier, true);
    }

    /**
     * Creates an instance of the object that will optionally return true the first time
     * hasChanged() is called.
     */
    public ChangeDetector(Supplier<V> valueSupplier, boolean forceInitialChange) {
      m_valueSupplier = valueSupplier;
      m_lastValue = valueSupplier.get();
      m_forceInitialChange = forceInitialChange;
    }

    public boolean hasChanged() {
      V currentVal = m_valueSupplier.get();
      boolean changed = false;
      if (((currentVal != null) && (m_lastValue != null))) {
        changed = m_forceInitialChange || !m_lastValue.equals(currentVal);
        m_forceInitialChange = false;
      }
      m_lastValue = currentVal;
      return changed;
    }
  }

  /** ChooserWithChangeDetection extends a SendableChooser to provide detection of changes */
  public static class ChooserWithChangeDetection<V> extends SendableChooser<V> {
    private ChangeDetector<V> m_changeDetector;

    /** Creates an empty chooser */
    public ChooserWithChangeDetection() {
      this(null, null, 0);
    }

    /**
     * Creates a chooser with specified titles and values and sets the initial choice
     *
     * @param choiceTitles Array of names of choices displayed in the chooser
     * @param choiceValues Values of choices displayed in the chooser. Must have the same dimension
     *     as choiceTitles
     * @param initialIndex index of the initial choice to display
     */
    public ChooserWithChangeDetection(String[] choiceTitles, V[] choiceValues, int initialIndex) {
      super();
      m_changeDetector = new ChangeDetector<V>(() -> this.getSelected());

      // Load options
      if ((choiceTitles != null) && (choiceValues != null)) {
        loadOptions(choiceTitles, choiceValues, initialIndex);
      }
    }

    /** Load the chooser with given titles and values and set the initial option */
    public void loadOptions(String[] choiceTitles, V[] choiceValues, int initialIndex) {
      for (int idx = 0; idx < choiceTitles.length; ++idx) {
        String title = choiceTitles[idx];
        V value = (idx < choiceValues.length) ? choiceValues[idx] : null;
        if ((title != null) && (value != null)) {
          this.addOption(title, value);
        }
      }

      // Set the initial option
      this.setDefaultOption(choiceTitles[initialIndex], choiceValues[initialIndex]);
    }

    public boolean hasChanged() {
      return m_changeDetector.hasChanged();
    }
  }

  /** ToggleButtonWithChangeDetection provides a dashboard toggle button with change detection */
  public static class ToggleButtonWithChangeDetection {
    private final SimpleWidget m_widget;
    private final GenericEntry m_netTableEntry;
    private final boolean m_defaultValue;
    private final ChangeDetector<Boolean> m_changeDetector;

    /**
     * Creates an instance of the object on a given dashboard tab
     *
     * @param tab Dashboard tab to add the widget to
     * @param title Title to give to the widget
     * @param defaultValue Default value of the widget
     */
    public ToggleButtonWithChangeDetection(
        ShuffleboardTab tab, String title, boolean defaultValue) {
      m_widget = tab.add(title, defaultValue);
      m_widget.withWidget(BuiltInWidgets.kToggleButton);
      m_netTableEntry = m_widget.getEntry();
      m_defaultValue = defaultValue;
      m_changeDetector =
          new ChangeDetector<Boolean>(() -> (m_netTableEntry.getBoolean(m_defaultValue)));
    }

    public boolean hasChanged() {
      return m_changeDetector.hasChanged();
    }

    public SimpleWidget getWidget() {
      return m_widget;
    }

    public boolean getValue() {
      return m_netTableEntry.getBoolean(m_defaultValue);
    }
  }

  /** SliderWithChangeDetection provides a dashboard slider with change detection */
  public static class SliderWithChangeDetection {
    private final SimpleWidget m_widget;
    private final GenericEntry m_netTableEntry;
    private final double m_defaultValue;
    private final ChangeDetector<Double> m_changeDetector;

    /**
     * Creates an instance of the object on a given dashboard tab
     *
     * @param tab Dashboard tab to add the widget to
     * @param title Title to give to the widget
     * @param defaultValue Default value of the widget
     * @param min Minimum value of the slider
     * @param max Maximum value of the slider
     */
    public SliderWithChangeDetection(
        ShuffleboardTab tab,
        String title,
        double defaultValue,
        double min,
        double max,
        double increment) {
      m_widget = tab.add(title, defaultValue);
      m_widget
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", min, "max", max, "Block increment", increment));

      m_netTableEntry = m_widget.getEntry();
      m_defaultValue = defaultValue;
      m_changeDetector =
          new ChangeDetector<Double>(() -> m_netTableEntry.getDouble(m_defaultValue), true);
    }

    public boolean hasChanged() {
      return m_changeDetector.hasChanged();
    }

    public SimpleWidget getWidget() {
      return m_widget;
    }

    public double getValue() {
      return m_netTableEntry.getDouble(m_defaultValue);
    }
  }

  /** A class that implements a dashboard panel for tuning PID values */
  public static class PIDTunerPanel {

    /** Width of the panel in cells */
    public static final int kPanelWidthCells = 8;

    /** Height of the panel in cells */
    public static final int kPanelHeightCells = 6;

    /** Width of a slider in the panel (cells) */
    public static final int kSliderWidth = kPanelWidthCells - 1;

    /** Height of a slider in the panel (cells) */
    public static final int kSliderHeight = 1;

    /** Internal PID controller used to provide a display panel */
    private final PIDController m_pid;

    /** Gain history for change detection */
    private PIDGains m_gains;

    /**
     * Creates an instance of the panel on a given tab
     *
     * @param tab Tab to create the panel on
     * @param title Title to display in the panel
     * @param rowIndex Row index (cells) where the panel should be positioned on the tab
     * @param colIndex Column index (cells) where the panel should be positioned on the tab
     */
    public PIDTunerPanel(
        ShuffleboardTab tab, String title, int rowIndex, int colIndex, PIDGains gains) {

      m_pid = new PIDController(gains.kP, gains.kI, gains.kD);
      m_gains = gains;

      tab.add(title, m_pid).withPosition(colIndex, rowIndex);
    }

    /** Returns the PID gins */
    public PIDGains getGains() {
      return new PIDGains(m_pid.getP(), m_pid.getI(), m_pid.getD());
    }

    /** Returns true if the PID gains have been modified since hasChanged() was last called */
    public boolean hasChanged() {
      boolean changed = false;
      PIDGains dashboardGains = new PIDGains(m_pid.getP(), m_pid.getI(), m_pid.getD());
      if (!m_gains.isEqual(dashboardGains)) {
        changed = true;
        m_gains = dashboardGains;
      }

      return changed;
    }
  }
}
