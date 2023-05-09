![FRC 5290 - VIKotics](../../../../../../../doc/graphics/5920-vikotics-logo_80x80.png "FRC 5290 - VIKotics")
**FRC 5290 - VIKotics**

---

# Auto-Builder: Generated Autonomous Routines

The AutoBuilder is a set of code modules that interpret several Dashboard user
interface control elements (called widgets) offering choices for where the robot
should move and what it should do during the autonomous period.  The values of
Dashboard widgets values are passed to AutoBuilder code modules, that generate
corresponding trajectories to move the robot along the selected path and combine
them with commands used to carry out other behaviors including shooting cargo
and balancing.

The Auto-Builder is intended to provide the robot drive team with a flexible set
of options during the autonomous period, enabling them to work alongside the
autonomous routines of other alliance robots.  Choice widgets used to build auto
routines include:

![Auto-Builder Chooser Widgets](./graphics/autobuilder-choice-widgets.jpg "Auto-Builder Chooser Widgets")

## Auto Choosers

### Auto Type
The Auto Type chooser selects between autonomous routines generated using the
Auto-builder and other [preset auto routines](./preset-auto-routines.md).  When
_AutoBuilder_ is selected, the other chooser widgets are used to determine the
specific actions taken during the generated auto routine.

### Initial Position
Sets the grid position (A-I) where the robot is located when the routine starts

### Initial Action
Specifies a way to shoot a pre-loaded cube into the grid

### Route
The route the robot travels to 


## References

- [2023 Layout Marking Diagram](https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf)
- [FRC 2023 Game animation](https://www.youtube.com/watch?v=0zpflsYc4PA&feature=youtu.be)
- [Charged Up Arena](https://firstfrc.blob.core.windows.net/frc2023/Manual/Sections/2023FRCGameManual-05.pdf)
- [Charged Up Match Play](https://firstfrc.blob.core.windows.net/frc2023/Manual/Sections/2023FRCGameManual-06.pdf)
