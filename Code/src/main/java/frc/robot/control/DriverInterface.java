package frc.robot.control;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Swerve.TeleOpSwerve;

public class DriverInterface {

    public int lastContorlState = 3;
    public int DesiredContorlState = 3;
    private Supplier<SwerveRequest> controlStyle;

    public void DriverContainer(){  
              
        //Triggers that check is desired control state equal current state if not set new contol state
        Trigger controlPick = new Trigger(() -> lastContorlState != getControlState());
        controlPick.onTrue(new InstantCommand(() -> newControlStyle()));

        //Set the directon the robot is facing to be the front of the feild
        JoystickButton btn_seed_feild_centic = new JoystickButton(RobotContainer.driverJoystick, 7);
        btn_seed_feild_centic.whileTrue(RobotContainer.drivetrain.runOnce(() -> RobotContainer.drivetrain.seedFieldCentric()));

        //Add one to disred contol state this should shiff the case into a robot centic request
        JoystickButton btn_robot_centic = new JoystickButton(RobotContainer.driverJoystick, 1);
        btn_robot_centic.onTrue(RobotContainer.drivetrain.runOnce(() -> DesiredContorlState=DesiredContorlState+1));
        btn_robot_centic.onFalse(RobotContainer.drivetrain.runOnce(() -> DesiredContorlState=DesiredContorlState-1));

        DoubleSupplier stopRotation;stopRotation = () -> RobotContainer.driverJoystick.getRawButton(9) ? 0.0 : 1.0;
       //New driver interface without clamp and new lever ramp range from 20%-100% commanded max speed
       new TeleOpSwerve(   
       () -> (RobotContainer.driverJoystick.getRawAxis(SwerveConstants.translationAxis)),
       () -> (RobotContainer.driverJoystick.getRawAxis(SwerveConstants.strafeAxis)),
       () -> -(RobotContainer.driverJoystick.getRawAxis(SwerveConstants.rotationAxis)* stopRotation.getAsDouble()),
       () -> (-(RobotContainer.driverJoystick.getRawAxis(SwerveConstants.sliderAxis)-1)/2.5+0.2)
        );
    }

    private int getControlState(){
        return DesiredContorlState;
     }

     private void newControlStyle() {
        lastContorlState = getControlState();
        switch (getControlState()) {
          case 1://This case uses normal CTRE control with the xBox joystick in feild centric
            controlStyle = () -> SwerveConstants.Feild_Centric_Driving
            .withVelocityX(RobotContainer.xBoxJoystick.getLeftY() * (SwerveConstants.MaxSpeed)) // Drive forward with negative Y (forward)
            .withVelocityY(RobotContainer.xBoxJoystick.getLeftX() * (SwerveConstants.MaxSpeed)) // Drive left with negative X (left)
            .withRotationalRate(RobotContainer.xBoxJoystick .getRightX() * (SwerveConstants.MaxAngularRate)); // Drive counterclockwise with negative X (left)
            break;
          case 2://This case uses normal CTRE control with the xBox joystick in robot centric
            controlStyle = () -> SwerveConstants.Robot_Centric_Driving
            .withVelocityX(RobotContainer.xBoxJoystick.getLeftY() * (SwerveConstants.MaxSpeed)) // Drive forward with negative Y (forward)
            .withVelocityY(RobotContainer.xBoxJoystick.getLeftX() * (SwerveConstants.MaxSpeed)) // Drive left with negative X (left)
            .withRotationalRate(RobotContainer.xBoxJoystick.getRightX() * (SwerveConstants.MaxAngularRate)); // Drive counterclockwise with negative X (left)
            break;
          case 3://This case uses custom control thought TeleOpSwerve in feild centric
            controlStyle = () -> SwerveConstants.Feild_Centric_Driving
            .withVelocityX(-TeleOpSwerve.getControl()[0] ) // Drive forward with negative Y (forward)
            .withVelocityY(-TeleOpSwerve.getControl()[1]) // Drive left with negative X (left)
            .withRotationalRate(TeleOpSwerve.getControl()[2]); // Drive counterclockwise with negative X (left)
            break;
          case 4:
          controlStyle = () -> SwerveConstants.Robot_Centric_Driving
          .withVelocityX(-TeleOpSwerve.getControl()[0]) // Drive forward with negative Y (forward)
          .withVelocityY(-TeleOpSwerve.getControl()[1]) // Drive left with negative X (left)
          .withRotationalRate(TeleOpSwerve.getControl()[2]); // Drive counterclockwise with negative X (left)
            break;
        }
        try {
          RobotContainer.drivetrain.getDefaultCommand().cancel();
        } catch(Exception e) {}
        RobotContainer.drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            RobotContainer.drivetrain.applyRequest(controlStyle).ignoringDisable(true));

            
      }
    
}
