// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TODO
 * 
 * Make sure all values are up to date not sure if this is an example of has already been run though out system
 * 
 * Thing to test
 * TeleOP Driving on x_box first learn what each button does and comment that. This way I can use them.
 * Robot on block is a good idea here.
 * 
 * after that test CTRE  and TeleOpSwerve and notice the differnce in contorl 
 * 
 * If robot is working as intended run camera method in robot period and see if odemetyr takes
 * 
 * If no camera or they work. Then let move to running autos. Create the auto that stated center and moved to each side as a base line.
 * Test PID to make sure they as good as they can be. then run center with each side and see how much more accrate 
 * then WPILib swerve was without cameras. then add cameras carefull if PID are wrong this will not be fun.
 * 
 * Please repost you finding to me anytime in this process. When Driving works thougt TeleOp swerve and you give it back to me with any 
 * changes and I will empliment selfdriving with Util class to make it easier to use and less math focused.
 * 
 * After that questNav will be introduct the code. I will keep an eye on the code beacsue it is changeing rapidly and new code 
 * that allows quest ot see tags drops  after champs which can me a big game changer and change the way we code it 
 * because it will not need photon ;)
 */

 package frc.robot;

 import static edu.wpi.first.units.Units.*;
 
 import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
 import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
 import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;
 import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
 import frc.robot.Constants.SwerveConstants;
 import frc.robot.commands.Swerve.TeleOpSwerve;
import frc.robot.commands.Swerve.ToPath;
import frc.robot.commands.Swerve.ToPose;
import frc.robot.control.DriverInterface;
import frc.robot.subsystems.SwerveBase.CommandSwerveDrivetrain;
 import frc.robot.subsystems.SwerveBase.Telemetry;
 import frc.robot.subsystems.SwerveBase.TunerConstants;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.util.SelfDriving.Pathfinder;
import frc.robot.control.*;

import frc.robot.Constants.SwerveConstants;
 
 public class RobotContainer {
    /*Intalizes Controller interface */
     public final static CommandXboxController xBoxJoystick = new CommandXboxController(0);
     public final static Joystick driverJoystick = new Joystick(0);

    /*Import contorl system that can be used */
     public static DriverInterface m_Driver  = new DriverInterface();
     public static TunningInterface m_Tunning  = new TunningInterface();

    /*Runs subsytems */
     public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
     private final Telemetry swerve_logger = new Telemetry(SwerveConstants.MaxSpeed);
     public static Pathfinder m_pathFinder  = new Pathfinder(drivetrain);
     public static AprilTagSubsystem m_aprilTags  = new AprilTagSubsystem();

     
/*PID auto tune stuff-
 * 
 * Run one of the sysID command 
 * Open the log file using this guide-https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
 * 
 */


     public RobotContainer() {
        //Runs the driver contol interface
        m_Driver.DriverContainer();

        //Runs the log for the swerve base
        drivetrain.registerTelemetry(swerve_logger::telemeterize);

        //Runs the command to be used in auto
        AutoCommands();


         
         //Change this method to try diffent swerve contorl
         //configureBindings_JoyStick_with_TeleOpSwerve();


        // JoystickButton btn_run_motor1 = new JoystickButton(driverJoystick, 1);
        //   btn_run_motor1.whileTrue(new ToPath("Example Path"));

       
     }
 
     public void AutoCommands(){
        NamedCommands.registerCommand("PathEnd", new RunCommand(()-> Pathfinder.PathEnded = true));
     }
 
     public Command getAutonomousCommand() {
             return AutoBuilder.buildAuto("New Auto");
     }
}