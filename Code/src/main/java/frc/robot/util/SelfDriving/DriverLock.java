package frc.robot.util.SelfDriving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveBase.Telemetry;

public class DriverLock { 
 public static double PoseDifferenceX;
 public static double PoseDifferenceY;
 public static double PoseDifferenceRotation;
 public static double PoseDifferenceRotationRaw;
 
 public static double XSpeed;
 public static double YSpeed;
 public static double RotationSpeed;
 
 public static double XSpeedFinal;
 public static double YSpeedFinal;
 public static double RotationSpeedFinal;
 
 public static boolean CloseEnough;
 public static double HowFar;
 public static double HowStraght;
 public static boolean TranslationClose;
 public static boolean RotationClose; 
 
 
 //Value requested will not go over this value
 public static double MaxTranslationSpeed = 0.25;//3.5
 public static double MaxRotationSpeed = Math.PI;
 public static double Y_X_Ratio = 4;
 public static double XCompensationValue = 1; 
 

 
 
 //Make the value request from the drive ramp the power to get to target
 //WARNING this could mess with deceleration if PID decelerates faster then is value
 //to fix issue lower this but would prefer for it to be high for smoother motion
 //Comment this out when first tuning PIDS
 /*Slew Limiter */
 static SlewRateLimiter MaxTranslationAccelerationX = new SlewRateLimiter(.55);
 static SlewRateLimiter MaxTranslationAccelerationY = new SlewRateLimiter(.55);
 static SlewRateLimiter MaxRotationAcceleration = new SlewRateLimiter(2*Math.PI);
 
//  // PIDs to Conrtol Driving to Note
//  public static PIDController NoteTranslation = new PIDController(0.285, 0, 0.0);
//  public static PIDController NoteRotation = new PIDController(0.325, 0, 0);
 
 // PIDs to Control Driving to Pose  
 public static PIDController MoveToPoseTranslation = new PIDController(0.3, 0.0, 0.0);
 public static PIDController MoveToPoseRotation = new PIDController(0.0125, 0, 0);




/*This is the logic being used for self driving as of 3-5-2025
 * 
 */
public static boolean AssumeControlX = false;
public static boolean AssumeControlY = false;
public static boolean AssumeControlRotation = false;


public static Pose2d currentPose2d(){
    return new Pose2d(new Translation2d(Telemetry.m_poseArray[0],Telemetry.m_poseArray[1]),new Rotation2d(Units.degreesToRadians(Telemetry.m_poseArray[2])));
}

public static void AssumeTotalControl (boolean State){
    AssumeControlX=State;  AssumeControlY=State;    AssumeControlRotation=State;
}

public static void AssumeControlX (boolean State){
    AssumeControlX=State;
}

public void AssumeControlY (boolean State){
    AssumeControlY=State;
}

public static void AssumeControlRotation (boolean State){
    AssumeControlRotation=State;
}
     //The following method needs to be called when self driving is used
 public static void DriveCalculationPose(double TargetPoseX,double TargetPoseY,double TargetPoseRotation) {
     //Because we are driving in feild centric for this command the number will change signs depending on allaince
   

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
         PoseDifferenceX = TargetPoseX - currentPose2d().getX();
         PoseDifferenceY = TargetPoseY - currentPose2d().getY();// Need to test might not be right
         PoseDifferenceRotationRaw = currentPose2d().getRotation().getDegrees() -TargetPoseRotation; // Need to test might not be right   
     }//TODO when testing make sure 180 offset no fuck shit up
         if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Red){
        //  PoseDifferenceX = SwerveBasePose.currentPoseX - TargetPoseX;
        //  PoseDifferenceY = SwerveBasePose.currentPoseY - TargetPoseY;// Need to test might not be right
        //  PoseDifferenceRotationRaw = SwerveBasePose.currentPoseRotation -TargetPoseRotation;// Need to test might not be right
        PoseDifferenceX = TargetPoseX - currentPose2d().getX();
        PoseDifferenceY = TargetPoseY - currentPose2d().getY();// Need to test might not be right
        PoseDifferenceRotationRaw = currentPose2d().getRotation().getDegrees() -TargetPoseRotation; // Need to test might not be right   
     }
 //Rotation should be the same regardless of alliance
 //The following will optimize the rotation to go left or right
         if (PoseDifferenceRotationRaw > 180){
         PoseDifferenceRotation = PoseDifferenceRotationRaw - 360;
     }
     //  if (PoseDifferenceRotationRaw < -180){
     //     PoseDifferenceRotation = PoseDifferenceRotationRaw + 360;
     // }
         else {
         PoseDifferenceRotation =  PoseDifferenceRotationRaw;
     }
 
   
     //Applies PID to move to Target Pose
     XSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceX, 0.0);
     YSpeed =  MoveToPoseTranslation.calculate(PoseDifferenceY, 0.0);
     RotationSpeed = MoveToPoseRotation.calculate(PoseDifferenceRotation, 0.0);
    
     
     //Sets a max output for variable
     XSpeed =  Math.abs(XSpeed) < MaxTranslationSpeed ?  XSpeed : MaxTranslationSpeed;
     YSpeed =  Math.abs(YSpeed) < MaxTranslationSpeed ?  YSpeed : MaxTranslationSpeed;
     RotationSpeed =  Math.abs(RotationSpeed) < MaxRotationSpeed ?  RotationSpeed : MaxRotationSpeed;
 

     //Apply the accelertion limiter 
     XSpeedFinal = MaxTranslationAccelerationX.calculate(XSpeed);
     YSpeedFinal = MaxTranslationAccelerationY.calculate(YSpeed);
     RotationSpeedFinal = MaxRotationAcceleration.calculate(RotationSpeed);
 
     // When CloseEnough == Fasle MoveToPose will end
     //This is where you will specify how close it must be for what you are doing to work
     if( (Math.abs(PoseDifferenceRotation)) < 5.0 && (Math.abs(PoseDifferenceY)) < 0.15 && (Math.abs(PoseDifferenceX)) < 0.15){
         CloseEnough = true;
     }
     else{
         CloseEnough = false;
     }
 
 

    } 
    
}
