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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
 import frc.robot.Constants.SwerveConstants;
 import frc.robot.commands.Swerve.TeleOpSwerve;
import frc.robot.commands.Swerve.ToPath;
import frc.robot.commands.Swerve.ToPose;
import frc.robot.subsystems.SwerveBase.CommandSwerveDrivetrain;
 import frc.robot.subsystems.SwerveBase.Telemetry;
 import frc.robot.subsystems.SwerveBase.TunerConstants;
 import frc.robot.util.SelfDriving.Pathfinder;

 
 public class RobotContainer {
     public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
          public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                
                    /* Setting up bindings for necessary control of the swerve drive platform */
                    public final static SwerveRequest.FieldCentric Feild_Centric_Driving = new SwerveRequest.FieldCentric()
                          //  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
             //.withSteerRequestType(SteerRequestType.MotionMagicExpo);
 
     public final SwerveRequest.RobotCentric Robot_Centric_Driving = new SwerveRequest.RobotCentric()
             .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1)
             .withSteerRequestType(SteerRequestType.MotionMagicExpo);
     
     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); 
     private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
 
     private final Telemetry swerve_logger = new Telemetry(MaxSpeed);
 
     private final CommandXboxController joystick = new CommandXboxController(0);
 
     public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 
     public final static Joystick driverJoystick = new Joystick(0);

     public static Pathfinder m_pathFinder  = new Pathfinder(drivetrain);

     
        
     public RobotContainer() {
         //Change this method to try diffent swerve contorl
         //configureBindings_JoyStick_with_TeleOpSwerve();

         JoystickButton btn_run_motor = new JoystickButton(driverJoystick, 1);
         //btn_run_motor.whileTrue(new ToPath("Example Path"));drivetrain.sysIdDynamic(Direction.kForward)
         btn_run_motor.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            

 
         drivetrain.registerTelemetry(swerve_logger::telemeterize);

                       //Stops the robot from reciving any rotation command
       DoubleSupplier stopRotation;stopRotation = () -> driverJoystick.getRawButton(9) ? 0.0 : 1.0;
       //New driver interface without clamp and new lever ramp range from 20%-100% commanded max speed
       
       new TeleOpSwerve(   
       () -> (driverJoystick.getRawAxis(SwerveConstants.translationAxis)),
       () -> (driverJoystick.getRawAxis(SwerveConstants.strafeAxis)),
       () -> -(driverJoystick.getRawAxis(SwerveConstants.rotationAxis)* stopRotation.getAsDouble()),
       () -> (-(driverJoystick.getRawAxis(SwerveConstants.sliderAxis)-1)/2.5+0.2),
       () -> !driverJoystick.getRawButton(1)// inverted=fieldCentric, non-inverted=RobotCentric
        );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            Feild_Centric_Driving.withVelocityX(-TeleOpSwerve.getControl()[0] * (MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(-TeleOpSwerve.getControl()[1]* (MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(TeleOpSwerve.getControl()[2] * (MaxAngularRate)) // Drive counterclockwise with negative X (left)
            ).unless(btn_run_motor)
        );
     
        
        NamedCommands.registerCommand("PathEnd", new RunCommand(()-> Pathfinder.PathEnded = true));

     }
 
     private void configureBindings_XBox() {
         // Note that X is defined as forward according to WPILib convention,
         // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(
             // Drivetrain will execute this command periodically
             drivetrain.applyRequest(() ->
             Feild_Centric_Driving.withVelocityX(joystick.getLeftY() * (MaxSpeed)) // Drive forward with negative Y (forward)
                     .withVelocityY(joystick.getLeftX() * (MaxSpeed)) // Drive left with negative X (left)
                     .withRotationalRate(joystick.getRightX() * (MaxAngularRate)) // Drive counterclockwise with negative X (left)
             )
         );
 
         joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
         joystick.b().whileTrue(drivetrain.applyRequest(() ->
             point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
         ));
 
         // Run SysId routines when holding back/start and X/Y.
         // Note that each routine should be run exactly once in a single log.
         joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
         joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
 
         // reset the field-centric heading on left bumper press
         joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
 
         drivetrain.registerTelemetry(swerve_logger::telemeterize);
     }
 
 
     private void configureBindings_JoyStick_with_CTRE() {
         // Note that X is defined as forward according to WPILib convention,
         // and Y is defined as to the left according to WPILib convention.
         drivetrain.setDefaultCommand(
             // Drivetrain will execute this command periodically
             drivetrain.applyRequest(() ->
             Feild_Centric_Driving.withVelocityX(driverJoystick.getRawAxis(1) * (MaxSpeed/4)) // Drive forward with negative Y (forward)
                     .withVelocityY(driverJoystick.getRawAxis(0) * (MaxSpeed/4)) // Drive left with negative X (left)
                     .withRotationalRate(driverJoystick.getRawAxis(2) * (MaxAngularRate/4)) // Drive counterclockwise with negative X (left)
             )
         );
     }
 
     private void configureBindings_JoyStick_with_TeleOpSwerve() {
              //Stops the robot from reciving any rotation command
       DoubleSupplier stopRotation;stopRotation = () -> driverJoystick.getRawButton(9) ? 0.0 : 1.0;
       //New driver interface without clamp and new lever ramp range from 20%-100% commanded max speed
       
       new TeleOpSwerve(   
       () -> (driverJoystick.getRawAxis(SwerveConstants.translationAxis)),
       () -> (driverJoystick.getRawAxis(SwerveConstants.strafeAxis)),
       () -> -(driverJoystick.getRawAxis(SwerveConstants.rotationAxis)* stopRotation.getAsDouble()),
       () -> (-(driverJoystick.getRawAxis(SwerveConstants.sliderAxis)-1)/2.5+0.2),
       () -> !driverJoystick.getRawButton(1)// inverted=fieldCentric, non-inverted=RobotCentric
        );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            Feild_Centric_Driving.withVelocityX(TeleOpSwerve.getControl()[0] * (MaxSpeed/4)) // Drive forward with negative Y (forward)
                    .withVelocityY(TeleOpSwerve.getControl()[1]* (MaxSpeed/4)) // Drive left with negative X (left)
                    .withRotationalRate(TeleOpSwerve.getControl()[2] * (MaxAngularRate/4)) // Drive counterclockwise with negative X (left)
            )
        );       
     }

 
     public Command getAutonomousCommand() {
             return AutoBuilder.buildAuto("New Auto");
     }
 }