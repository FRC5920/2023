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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
      boolean changed = m_lastValue != currentVal;
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
      m_changeDetector = new ChangeDetector<V>(() -> this.getSelected(), true);

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
}
