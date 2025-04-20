package frc.robot.util.Control;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonGroup extends SubsystemBase {
    private int lastActiveIndex = -1; // Track last active button
    private GenericEntry[] buttons;
    private String[] ButtonNames;

    public ButtonGroup(ShuffleboardTab tab, String[] ButtonNames) {
        this.ButtonNames=ButtonNames;

        for (int i = 0; i < ButtonNames.length; i++) {
            // ShuffleboardTab tab = Shuffleboard.getTab("Controls-Arm");
            buttons[i] = tab.add(ButtonNames[i], false).withWidget("Toggle Button").getEntry();
        }

        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                buttons[i].setBoolean(true); // Set first available button ON
                lastActiveIndex = i;
                break;
            }
        }
    }

    private int getChangedButtonIndex() {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                boolean currentState = buttons[i].getBoolean(false);
                if (currentState && i != lastActiveIndex) { // Detect new button press
                    return i;
                }
            }
        }
        return -1;
    }

    private void resetOtherButtons(int activeIndex) {
        for (int i = 0; i < buttons.length; i++) {
            if (buttons[i] != null) {
                buttons[i].setBoolean(i == activeIndex); // Keep only one button active
            }
        }
    }

    public int getActiveButtonIndex() {
        return lastActiveIndex;
    }

    public String getActiveButtonName() {
        if (lastActiveIndex >= 0 && lastActiveIndex < ButtonNames.length) {
            return ButtonNames[lastActiveIndex];
        }
        return "None";
    }

    public Trigger getDashboardEntryAsTrigger(int i){
      return new Trigger(()-> buttons[i].getBoolean(false));
    }

    @Override
    public void periodic() {
        int newActiveIndex = getChangedButtonIndex();

        if (newActiveIndex != -1 && newActiveIndex != lastActiveIndex) {
            resetOtherButtons(newActiveIndex);

            if (buttons[newActiveIndex] != null) {
                buttons[newActiveIndex].setBoolean(true);
                lastActiveIndex = newActiveIndex;
            }
        }
    }

}
