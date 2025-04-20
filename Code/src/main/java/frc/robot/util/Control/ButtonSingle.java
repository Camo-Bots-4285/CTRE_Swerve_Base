package frc.robot.util.Control;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonSingle {

    // ShuffleboardTab tab = Shuffleboard.getTab("Controls-Arm");
    public static GenericEntry createButton(ShuffleboardTab tab, String Title){
        GenericEntry dashboardEntry = tab.add(Title, false).withWidget("Toggle Button").getEntry();
        return dashboardEntry;
    }

    public static Trigger getDashboardEntryAsTrigger(GenericEntry dashboardEntry){
      return new Trigger(()-> dashboardEntry.getBoolean(false));
    }

    public void setButtonState(GenericEntry dashboardEntry,boolean state){
        dashboardEntry.setBoolean(state);
    }

}
