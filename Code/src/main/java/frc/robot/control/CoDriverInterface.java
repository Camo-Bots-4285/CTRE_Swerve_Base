package frc.robot.control;



import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.Control.ButtonGroup;
import frc.robot.util.Control.ButtonSingle;

public class CoDriverInterface {
    public static SendableChooser<String> mChooser;

    public CoDriverInterface(){}

    /*The following shows hoe to us button group to run command keep in mind you can also pull the index and string name 
     * to a subsytem if writng out commands for button is not oprimal. Group button are to be use when you have a group and you 
     * only want one button active at a time.
      */
    //First define the tab the button will be place in
    private ShuffleboardTab tab = Shuffleboard.getTab("TeleOp");
    //Define the name that will be displayed
    private String[] ExampleNames = {"BOB","BOB Jr","BOB The Third"};

    //Use the ButtonGroup file from util to run code
    public final ButtonGroup example = new ButtonGroup(
        tab,
        ExampleNames
      );

    int little_bob;//Place holder


    public void CoDrivercontainer(){
        //Check if button BOB it triggered and if so set little_bob equal to the currect active index which would be zero
        //This is just a place holder not meant to do anything
        example.getDashboardEntryAsTrigger(0).onTrue(new InstantCommand(() -> little_bob=example.getActiveButtonIndex()));

        //Creates a button and use it as a command in one line
        ButtonSingle.getDashboardEntryAsTrigger(ButtonSingle.createButton(tab, "Single Button")).onTrue(new InstantCommand(() -> little_bob=example.getActiveButtonIndex()));
    

        //Here is a example of an mchooser there is no util file for it it needed there can be
         mChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Choices", mChooser);
        mChooser.setDefaultOption("Straight", "Straight");
        mChooser.addOption("Center", "Center");

        //The following code wil tereturn the choosen sting
        //mChooser.getSelected();
    }

    
}
