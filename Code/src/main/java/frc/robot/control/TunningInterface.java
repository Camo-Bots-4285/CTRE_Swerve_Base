package frc.robot.control;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Swerve.TeleOpSwerve;

public class TunningInterface {

    public void TunnerContainer(){

        //This will run the sysID routine for the typr selelected in CommandSwervedrivtrain
        // It can tune drive motor, rotation motor, and face controler
         JoystickButton btn_quasisatic_forward = new JoystickButton(RobotContainer.driverJoystick, 3);
         btn_quasisatic_forward.whileTrue(RobotContainer.drivetrain.sysIdQuasistatic(Direction.kForward));

         JoystickButton btn_quasisatic_reverse = new JoystickButton(RobotContainer.driverJoystick, 4);
         btn_quasisatic_reverse.whileTrue(RobotContainer.drivetrain.sysIdQuasistatic(Direction.kReverse));
            
         JoystickButton btn_dynamic_forward = new JoystickButton(RobotContainer.driverJoystick, 5);
         btn_dynamic_forward.whileTrue(RobotContainer.drivetrain.sysIdDynamic(Direction.kForward));

         JoystickButton btn_dynamic_reverse = new JoystickButton(RobotContainer.driverJoystick, 6);
         btn_dynamic_reverse.whileTrue(RobotContainer.drivetrain.sysIdDynamic(Direction.kReverse));

    }
    
}
