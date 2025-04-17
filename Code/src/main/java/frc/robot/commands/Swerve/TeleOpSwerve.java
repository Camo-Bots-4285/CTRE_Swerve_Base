package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveBase.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveBase.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

import static edu.wpi.first.units.Units.*;

//import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;

public class TeleOpSwerve {
  /*
   * Teleoperated Swerve Drive Command
   * ---------------------------------
   * 
   * This command hooks up to the Swerve Drive subsystem
   * and passes in our joystick inputs into it.
   */

  /*
   * Joysticks return DoubleSuppliers when the get methods are called
   * This is so that joystick getter methods can be passed in as a parameter but
   * will continuously update,
   * versus using a double which would only update when the constructor is called
   */
  private static DoubleSupplier forwardX;
  private static DoubleSupplier forwardY;
  private static DoubleSupplier rotation;
  private static DoubleSupplier speedchange;
  private static Supplier<Boolean> isFieldOriented;

  private final static SlewRateLimiter xLimiter = new SlewRateLimiter(
      SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final static SlewRateLimiter yLimiter = new SlewRateLimiter(
      SwerveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final static SlewRateLimiter turningLimiter = new SlewRateLimiter(
      SwerveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  public static double fwdX;
  public static double fwdY;
  public static double rot;
  public static boolean fieldCentric;
  public static double AdjustDriveSpeed;

  public TeleOpSwerve(
      DoubleSupplier fwdX,
      DoubleSupplier fwdY,
      DoubleSupplier rot,
      DoubleSupplier AdjustDriveSpeed,
      Supplier<Boolean> isFieldOriented) {

    forwardX = fwdX;
    forwardY = fwdY;
    rotation = rot;
    speedchange = AdjustDriveSpeed;

    this.isFieldOriented = isFieldOriented;

  }

  public static double[] getControl() {

    AdjustDriveSpeed = speedchange.getAsDouble();
    /*
     * Units are given in meters per second radians per second. Since joysticks give
     * output
     * from -1 to 1, we multiply the outputs by the max speed. Otherwise, our max
     * speed
     * would be only 1 meter per second and 1 radian per second.
     */

    fwdX = forwardX.getAsDouble();
    fwdY = forwardY.getAsDouble();
    rot = rotation.getAsDouble();
    fieldCentric = isFieldOriented.get();

    // 2. Apply deadband/deadzone, can edit this later to have smoother behavior
    // If velocity is less then number it will be set to zero need to tune these
    // value with driver
    fwdX = Math.abs(fwdX) > 0.05 ? fwdX : 0.0;
    fwdY = Math.abs(fwdY) > 0.05 ? fwdY : 0.0;// 0.1
    rot = Math.abs(rot) > .30 ? rot : 0.0;

    // 3. Make the driving smoother this will set max velocity in teleop
    // There should be three setting that are programed in normal and other two are
    // activated by buttons
    // superfast and superslow(should be pared with high amps if push needed)
    fwdX = xLimiter.calculate(fwdX * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond * AdjustDriveSpeed)/(SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    fwdY = yLimiter.calculate(fwdY * SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond * AdjustDriveSpeed)/(SwerveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    rot = turningLimiter.calculate(rot * SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * AdjustDriveSpeed)/(SwerveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

    double[] States = {fwdX,fwdY,rot};

    return States;

  }
}
