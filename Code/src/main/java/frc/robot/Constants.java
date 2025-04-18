package frc.robot;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotContainer;



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

 

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
 public final class Constants {

    public static final class SwerveConstants {
    /* Drive Controls */
    public static final int translationAxis = 1;
    public static final int strafeAxis = 0;
    public static final int rotationAxis = 2; // was 4 on Xbox
    public static final int sliderAxis = 3;

    /* Drivetrain Constants */// Measured from center of each module (wheel axis)
    public static final double trackWidth = Units.inchesToMeters(22.625); 
    public static final double wheelBase = Units.inchesToMeters(22.625);
    public static final double robotRotationFactor = Math.sqrt(trackWidth*trackWidth + wheelBase*wheelBase);
    public static final double robotMass = 90;//kg
    public static final double centerofGravity = 0.12;//meter

    /* Wheel Diameter *///Try to get as accurate as possible to reduce error 
    public static final double wheelDiameter = Units.inchesToMeters(4); 
    public static final double wheelCircumference = wheelDiameter * Math.PI;


    /* Gear Ratio */
    public static final double driveGearRatio = 6.12; // Mk3 drive ratio
    public static final double angleGearRatio = 12.8; // Mk3 steer ratio   


        /* Swerve Profiling Values these value are all therotical if robot is not
     * tacking as you wish or not finctioing properly please lower these number
     * Also any time there is an update to the swerve base these value will need to be rcalibrated
     */

     public static final double maxSpeed = 5.0;//In meter/Second  //Hypathetically 5.21208
     public static final double maxAngularVelocity = maxSpeed/robotRotationFactor; //meter/sec to radian/sec
     public static final double maxAcceleration = 3.5;//Hypethicaly 6.2// Motor_MAX_Tork/(robotMass*Math.pow(centerofGravity,2.0))
     
     
    /*Motor Constaints */
    public static final double Drive_Motor_MAX_RPS = maxSpeed*driveGearRatio/wheelCircumference;//RPM/60=RPS
    public static final double Drive_Motor_MAX_Accleration = maxAcceleration*driveGearRatio/wheelCircumference;//RPM/60=RPS
    public static final double Drive_Motor_MAX_Jerk =0;    

    /*Encoder Id's + Pigeon */
    public static final int frontLeftRotationEncoderId = 3;
    public static final int frontRightRotationEncoderId = 2;
    public static final int rearLeftRotationEncoderId = 4;
    public static final int rearRightRotationEncoderId = 1;
    
    public static final int PIGEON_SENSOR_ID = 0;

    /*Spark/Talon Id's */
    public static final int frontLeftRotationMotorId = 21;
    public static final int frontLeftDriveMotorId = 11;

    public static final int frontRightRotationMotorId = 22;
    public static final int frontRightDriveMotorId = 12;

    public static final int rearLeftRotationMotorId = 23;
    public static final int rearLeftDriveMotorId = 13;

    public static final int rearRightRotationMotorId = 24;
    public static final int rearRightDriveMotorId = 14;

    /* Set Normal Pigeon of Set */
    public static final int NormalPigeonOfSet = 0;

    /* TeleOp Swerve Constants *///Tune a little higher then what driver is confetable driving for Fast and normal
    public static double kTeleDriveMaxSpeedMetersPerSecondFast = maxSpeed;//Try getting this value from Choreo
    public static double kTeleDriveMaxSpeedMetersPerSecondNormal = 2.0; 
    public static double kTeleDriveMaxSpeedMetersPerSecondSlow = 0.125;   

    public static double kTeleDriveMaxSpeedMetersPerSecond = kTeleDriveMaxSpeedMetersPerSecondNormal;  
    public static double kTeleDriveMaxAccelerationUnitsPerSecond = maxAcceleration;//Try getting this value from Pathplanner
    
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kTeleDriveMaxSpeedMetersPerSecond/robotRotationFactor;//Try getting this value from Choreo
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = maxAcceleration/robotRotationFactor;//Try getting this value from Choreo


    /* Swerve Calibration
    Now after all that set up we need to make sure the Swerve Base gets calibrated correctly. The following goes over how
    to calibrate a bot who center of gravity is in the center of the robot

    NOTE: If Robots Center is gravity is not center of the wheels
    Thought technically weight should have nothing to do with the movement of a module, it can add friction. I have added a system 
    to acount for this called Weight/SwerveDriveModule. If you find conventianl tuneing is not working as well as you hoped it would.
    Uncomment everything that says "needed for Weight/SwerveDriveModule". Keep in mind this is all theoretical and has not been test 
    as of (8/14/2024) so system might need a few changes like changes in signs to become fully functioning.

    1) Edit(DrivePID1)/Make a button that uses the drive command at a set speed (perferably speed for auto) make sure calibrationFactorSB = 1.0
    Lift robot up and tune PIDs for free spin (PIDs for Indivual Motors) PID is tuned when SmartDashBoard ## wheel Speed is withing fice decimal
    places of the taget value

    2) Choose multiple other speeds and record taget(x) vs actual-target(y) without changing PIDS use this data to create an equation to 
    add to DriveController kP in swerve module

    Equation formate:
    Linear: Slope*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)
    Quadratic: A(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)*(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed) + B(optimizedDesiredState.speedMetersPerSecond * SwerveConstants.maxSpeed - auto speed)

    3) When you are satisfied with the tracking place to robot on the ground and find the percent error and make the inverese equal to calibrationFactorSB 
   
    4) Now it is final time to start tuning auto!! Go to "Set PIDs for Path Planner" in Swerve Base.
    
    */

    /*PIDs for set Voltage*/
    // public static double frontLeft_Drive_kP = 0.3487;//0.3487//.38
    // public static double frontLeft_Rotation_kP = 0.5;

    // public static double frontRight_Drive_kP = 0.3464;//0.3464//.388
    // public static double frontRight_Rotation_kP = 0.5;

    // public static double rearLeft_Drive_kP = 0.3447;//0.3447//.38
    // public static double rearLeft_Rotation_kP = 0.5;

    // public static double rearRight_Drive_kP = 0.3473;//0.3473.3945
    //  public static double rearRight_Rotation_kP = 0.5;

    /**PIDs for Indivual Motors 
     * 
     * While tuning first start with FF you can use the following links to 
     * help get a starting value try get get all variable as close as posible
     * 
     * Drive-https://www.reca.lc/drive
     *  When you use this cite please copy and pasta link to your project below so there may see the values you used
     *
     * Rotation- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-turret.html
     * You can read thought the following website and try to implment a FF contollor but as long as the
     * wheel turn quickly in the durection we want and stay there this is a little less import that does not mean
     * do not tak your time and make sure it is right.
     */
    public static PIDController frontLeft_Drive_PID = new PIDController(0.0, 0.0, 0.003);//14.1207
    public static PIDController frontRight_Drive_PID = new PIDController(0.0,0.0, 0.003);//14.1125
    public static PIDController rearLeft_Drive_PID = new PIDController(0.0, 0.0, 0.003);//14.347
    // Changed to test
    public static PIDController rearRight_Drive_PID = new PIDController(0.0, 0.0, 0.003);
    //public static PIDController rearRight_Drive_PID = new PIDController(.219, 0.0, 0.00085432);//14.18//1.5//1.7//0.5, 0.0,0.00195052
   //0.00901141 to much
   //0.0090114 good ish  -6.5 2.31, 0.0, 0.0090114
   //0.0090112 to little
   
    //It seem that kp and kv should be the same value
    //How ever kd has to offset the uncertenty of kp so the wheel does not occilate. 0.0, 0.0, 0.0

    public static SimpleMotorFeedforward frontLeft_Drive_FF = new SimpleMotorFeedforward(0.145,0.12,0.0025);// 0.16,2.2925,0.0
    public static SimpleMotorFeedforward frontRight_Drive_FF = new SimpleMotorFeedforward(0.145,0.11875,0.0025);// 0.16,2.27,0.0
    public static SimpleMotorFeedforward rearLeft_Drive_FF = new SimpleMotorFeedforward(0.145,0.1185,0.0025);// 0.16,2.265,0.0
    public static SimpleMotorFeedforward rearRight_Drive_FF = new SimpleMotorFeedforward(0.145,0.1185,0.0025);//0.155,2.26,0.0

    public static PIDController frontLeft_Rotation_PID = new PIDController(5, 0.0, 0.0);
    public static PIDController frontRight_Rotation_PID = new PIDController(5, 0, 0);
    public static PIDController rearLeft_Rotation_PID = new PIDController(5, 0, 0);
    public static PIDController rearRight_Rotation_PID = new PIDController(5, 0, 0);

    public static SimpleMotorFeedforward frontLeft_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward frontRight_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward rearLeft_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    public static SimpleMotorFeedforward rearRight_Rotation_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);


/*
 * Step-by-Step PID and FeedForward Tuning Plan for Robot Movement
 * 
 * 1. **Set Initial Values:**
 *    - Start with zero for all PID and FeedForward gains:
 *      - P = 0, I = 0, D = 0
 *      - FeedForward velocity = 0, FeedForward acceleration = 0
 *      - Static FeedForward (`ks`) = 0
 * 
 * 2. **Tune Static FeedForward (`ks`) for Low-Speed Movement:**
 *    - `ks` compensates for static friction and ensures the robot can overcome it and start moving.
 *    - Set `ks` to a small value (e.g., `ks = 0.01`), then gradually increase it if the robot hesitates or stalls at low speeds.
 *    - The goal is to get the robot to move smoothly at low speeds without stuttering or excessive torque.
 *    - Adjust `ks` until the robot starts moving from rest with smooth and controlled acceleration.
 * 
 * 3. **Tune FeedForward (Velocity and Acceleration):**
 *    - After static friction is overcome, tune the FeedForward velocity and acceleration gains.
 *    - Start by adjusting the FeedForward velocity term (e.g., from 0.1 to 0.2), testing the robot’s response to varying speeds.
 *    - Adjust FeedForward acceleration for smoother starts from a standstill.
 *    - The robot should accelerate and decelerate smoothly without oscillations or large delays.
 * 
 * 4. **Tune Proportional Gain (P):**
 *    - Set I = 0 and D = 0, and focus on tuning **P** first to control the robot’s responsiveness.
 *    - Start with a small P-gain (e.g., P = 0.1) and gradually increase it to eliminate sluggishness and improve speed accuracy.
 *    - Ensure there’s no overshoot or oscillation; if overshoot happens, reduce P.
 * 
 * 5. **Tune Integral Gain (I):**
 *    - After adjusting **P**, set D = 0 and tune **I** to minimize steady-state error (small persistent errors).
 *    - Start with a small value for **I** (e.g., I = 0.01) and increase it until any small drift is corrected.
 *    - Watch for wind-up (large I value), which could cause overshoot or instability.
 * 
 * 6. **Tune Derivative Gain (D):**
 *    - Set **I = 0** and focus on **D** to smooth out any overshoot or oscillations caused by **P**.
 *    - Gradually increase **D** to stabilize the motor response.
 *    - If the robot becomes sluggish or noisy, reduce **D** until smooth control is achieved.
 * 
 * 7. **Test on Ground and Adjust for Motor Interaction:**
 *    - After tuning off-ground, test the robot on the ground to account for real-world friction and weight distribution.
 *    - Fine-tune **FeedForward velocity** and **acceleration** as necessary for optimal on-ground performance.
 *    - If motors interfere with each other (e.g., one motor affects the other’s response), adjust the PID values to ensure balanced performance.
 * 
 * **To Make Tuning Values Work on the Ground:**
 *    - The **ground effect** changes motor behavior due to increased friction, weight distribution, and possible coupling between the wheels.
 *    - **Start with the off-ground tuned values** (such as `ks`, FeedForward, and PID gains) and test them on the ground.
 *    - **If the robot is too slow to start** or struggles to move, increase **`ks`** slightly to compensate for the additional friction of the ground.
 *    - **FeedForward adjustments** might also be necessary:
 *      - If the robot accelerates too slowly or drags, increase **FeedForward velocity** and **FeedForward acceleration**.
 *      - If the robot accelerates too quickly or jerks, slightly decrease the FeedForward values.
 *    - **PID gains** may also need slight adjustments:
 *      - If the robot oscillates or overshoots, reduce **P** or increase **D**.
 *      - If the robot drifts or struggles to hold a consistent speed, increase **I** to reduce steady-state error.
 *    - Keep an eye on **motor interaction**; if one motor is affecting the other (e.g., one motor is not reacting as expected due to ground friction or loading), adjust the PID parameters for each motor individually, if your system allows for that.
 *    - For **two-wheel drive robots** or other setups, sometimes adjusting the left and right motor PID terms separately is necessary to account for slight asymmetries in wheel friction or motor response.
 * 
 * 8. **Verify High-Speed Performance:**
 *    - After fine-tuning at low speeds, test the robot at higher speeds (e.g., 50%-100% of max speed).
 *    - Ensure the robot accelerates, decelerates, and holds a straight path without oscillations or excessive overshoot.
 * 
 * 9. **Repeat as Needed:**
 *    - Revisit any of the previous steps if issues arise in different conditions (e.g., varying surfaces or loads).
 * 
 * By following this approach, you start by tuning **static FeedForward (`ks`)** first, ensuring smooth low-speed movement before focusing on dynamic control with **PID** and **FeedForward** terms. The final tuning step of testing on the ground ensures that real-world factors such as friction, weight distribution, and motor interaction are accounted for.
 */




    /* Calibration Factor to Help offest for weight start with this at one*/
    public static double calibrationFactorSB = 1.0;//1.11


    //The following should not be touch 
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // front left, ++ quadrant
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right, +- quadrant
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left, -+ quadrant
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right, -- quadrant
    );

    public static Translation2d mDriveRadius = new Translation2d(trackWidth/2, wheelBase/ 2);
    
  }

    public static final class VisionConstants {

    //TODO Calculate and get these values, Units need to be in meters and radians
    /**
     * Physical location of the apriltag camera on the robot, relative to the center
     * of the robot.
     */

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_1 = new Transform3d(
    //     new Translation3d(-0.063, -0.3125, 0.562),// Get from CAD Model In meters-0.063, -0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-45.0)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_2 = new Transform3d(
    //     new Translation3d(0.063, -0.3125, 0.562),//0.063, -0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_3 = new Transform3d(
    //     new Translation3d(0.063, 0.3125, 0.562),//0.063, 0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(135)));

    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(-0.063, 0.3125, 0.562),//-0.063, 0.3125, 0.562
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(45)));
    
    //These are updated camera positions for 2025
    public static final Transform3d ROBOT_TO_APRILTAG_CAMERA1 = new Transform3d(  
      new Translation3d(Units.inchesToMeters(-7.335), Units.inchesToMeters(13.220), Units.inchesToMeters(5.915)), // Get from CAD Model In meters -13.220
      new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(60)));


  public static final Transform3d ROBOT_TO_APRILTAG_CAMERA2 = new Transform3d(
      new Translation3d(Units.inchesToMeters(-13.220), Units.inchesToMeters(7.335), Units.inchesToMeters(5.915)),//0.063, -0.3125, 0.562
      new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(-150)));


  public static final Transform3d ROBOT_TO_APRILTAG_CAMERA3 = new Transform3d(
      new Translation3d(Units.inchesToMeters(13.220), Units.inchesToMeters(-7.335), Units.inchesToMeters(5.915)),//0.063, 0.3125, 0.562
      new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(30)));


  public static final Transform3d ROBOT_TO_APRILTAG_CAMERA4 = new Transform3d(
      new Translation3d(Units.inchesToMeters(-7.335), Units.inchesToMeters(-13.220), Units.inchesToMeters(5.915)),//-0.063, 0.3125, 0.562
      new Rotation3d(0.0, Units.degreesToRadians(-35), Units.degreesToRadians(-60)));//-225

    //Lime Light 
    //          CAD             RealLife   OldVaules 
    // Height  0.6673348        0.684      0.669
    //Front    0.792804         0.072      0.059
    //Right    0.3028135        0.311      0.303
    //Angle    29.7169722 Deg   30 Deg     30 deg
    // public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_4 = new Transform3d(
    //     new Translation3d(0.3109483, 0.0631137, -0.567547072),
    //     new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-135.0)));
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT_5 = new Transform3d(
        new Translation3d(0.0, 0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(0)));
    
        //Main Note Camera Assumed to be center 
     public static final Transform3d Note_Camera_Main_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0),15.0, Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant1_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));

      public static final Transform3d Note_Camera_Assistant2_To_Robot = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),//0.072, -0.311, 0.669
        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));
    
    
    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.2042;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
      new Translation2d(FIELD_LENGTH_METERS , FIELD_WIDTH_METERS),
      new Rotation2d(Math.PI)
    );

    

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    //Raise this value to reject less accurate poses 0.2 recommended by photon vision
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class REV_Motor_Charactoristics{

    public static final int[] Neo_int={
      0, //PLeace hold for motor so number line up
      40, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      7, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      60  //4 Emergency current limit
    };
    public static final double[] Neo_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    public static final int[] Neo550_int={
      0,//PLeace hold for motor so number line up
      35, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      5, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      50  //4 Emergency current limit
    };
    public static final double[] Neo550_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    public static final int[] NeoVortex_int={
      0,//PLeace hold for motor so number line up
      40, //1 Stall Current Limit: The current limit in Amps at 0 RPM.
      5, //2 Free spin: The current limit at free speed (5700RPM for NEO).
      0, //3 LimitRPM: RPM less than this value will be set to the stallLimit, RPM values greater than limitRpm will scale linearly to freeLimit
      55  //4 Emergency current limit
    };
    public static final double[] NeoVortex_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Place holder for gear ratio
      0, //1  Plcae holder for wheel radius
      75,//2 Tempurature OverHeat Warning will call in celcius
    };
    
    
    }
 
    //Here is an example of how to set up constats for a Neo 550
  public static class REV_Example{

    /*Motor location */
    public static final String MotorIdentification = "PlaceHolderName";//Change this to a helpful identify the motor when priting values

    public static final int[] Motor_int ={
    1, //0 Motor ID
    REV_Motor_Charactoristics.Neo550_int[1], //1 Stall current limit
    REV_Motor_Charactoristics.Neo550_int[2], //2 FreeSpin 
    REV_Motor_Charactoristics.Neo550_int[3], //3 LimitRPM
    REV_Motor_Charactoristics.Neo550_int[4]  //4 Emergency current limit
    };

    /* Motion Tracking Constants*/
    public static final double[] Motor_double = {//Start counting at zero and every number after that is positive plus one
      0, //0  Gear Ratio
      0, //1  Wheel Radius [meters]
      REV_Motor_Charactoristics.Neo_double[2]//2 Tempurature OverHeat Warning will call
    }; 

    public static final boolean[] Motor_boolean = {//Start counting at zero and every number after that is positive plus one
      //Break mode is true 
      true, //0  Active Motor State
      false, //1 Invert Motor

    }; 

    public static final double[] TunnerValues = {
      0, //0  Position PID kP
      0, //1  Position PID kI
      0, //2  Position PID kD
      0, //3  Position FF  kV
      0, //4  Position iZone
      0, //5  Position iMaxAccumulation
      0, //6  Position OutPut Min
      0, //7  Position OutPut Max


      0, //8  Velocity PID kP
      0, //9  Velocity PID kI
      0, //10  Velocity PID kD
      0, //11  Velocity FF  kV
      0, //12  Velocity iZone
      0, //13  Velocity iMaxAccumulation
      0, //14  Velocity OutPut Min
      0, //15  Velocity OutPut Max

      0, //16  Velocity PID kP
      0, //17  Velocity PID kI
      0, //18  Velocity PID kD
      0, //19  Velocity FF  kV
      0, //20  Velocity iZone
      0, //21  Velocity iMaxAccumulation
      0, //22  Velocity OutPut Min
      0, //23  Velocity OutPut Max
 
    };

    public static final double[] MotionConstants = {
      0, //0 max Velocity
      0, //1 max Acceleration
      0, //2 allowed Closed Loop Error
    };

  }

  public static class CTRE_Motor_Charactoristics{

    public static final int[] Kraken60_int ={
      0, //0 Pleace holder for motor ID
      50, //1 Stator Current Limit max amps for the motor
      55, //2 SupplyCurrentLimit pulls this till lower currnt is applies
      40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
      70  //4 Emergency Stop Current
    
      };
      public static final double[] Kraken60_double = {//Start counting at zero and every number after that is positive plus one
        0, //0  Place holder for Gear Ration
        0, //1  Pleace holder for Wheelradius
        75,//2 Tempurature OverHeat Warning will call
        0.75//3 Supply Curren Time amount of time motor pulls max amps
      }; 

      public static final int[] Kraken44_int ={
        0, //0 Pleace holder for motor ID
        50, //1 Stator Current Limit max amps for the motor
        45, //2 SupplyCurrentLimit pulls this till lower currnt is applies
        40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
        60  //4 Emergency Stop Current
      
        };
        public static final double[] Kraken44_double = {//Start counting at zero and every number after that is positive plus one
          0, //0  Place holder for Gear Ration
          0, //1  Pleace holder for Wheelradius
          0,//2 Tempurature OverHeat Warning will call
          0//3 Supply Curren Time amount of time motor pulls max amps
        }; 

        public static final int[] Falcon_int ={
          0, //0 Pleace holder for motor ID
          50, //1 Stator Current Limit max amps for the motor
          45, //2 SupplyCurrentLimit pulls this till lower currnt is applies
          40, //3 SupplyCurrentLimtLower continous current should not be higher then breaker
          0  //4 Emergency Stop Current
        
          };
          public static final double[] Falcon_double = {//Start counting at zero and every number after that is positive plus one
            0, //0  Place holder for Gear Ration
            0, //1  Pleace holder for Wheelradius
            0,//2 Tempurature OverHeat Warning will call
            0//3 Supply Curren Time amount of time motor pulls max amps
          };  
    
 
    }

    //The following code it an example for a kraken 60
  public static class Kraken_Motor{

 
  public static final String CANLoop = "rio";// Idenify which CAN loop motor is on
  public static final String MotorIdentification = "PlaceHolderName";//Change this to a helpful identification name


  public static final int[] Motor_int ={
  1, //0 Motor ID
  CTRE_Motor_Charactoristics.Kraken60_int[1], //1 Stator Current Limit max amps for the motor
  CTRE_Motor_Charactoristics.Kraken60_int[2], //2 SupplyCurrentLimit pulls this till lower currnt is applies
  CTRE_Motor_Charactoristics.Kraken60_int[3],  //3 SupplyCurrentLimtLower continous current should not be higher then breaker
  CTRE_Motor_Charactoristics.Kraken60_int[4]  //4 Emergency Stop Current

  };

  /* Motion Tracking Constants*/
  public static final double[] Motor_double = {//Start counting at zero and every number after that is positive plus one
    0, //0  Gear Ratio
    0, //1  Wheel Radius [meters]
    CTRE_Motor_Charactoristics.Kraken60_double[2],//2 Tempurature OverHeat Warning will call
    CTRE_Motor_Charactoristics.Kraken60_double[3]//3 Supply Curren Time amount of time motor pulls max amps
  }; 

  public static final boolean[] Motor_boolean = {//Start counting at zero and every number after that is positive plus one
    //Break mode is true 
    true, //0  Active Motor State
    false, //1 Invert Motor

  }; 

  public static final double[] TunnerValues = {
    0, //0  Position PID kP
    0, //1  Position PID kI
    0, //2  Position PID kD
    0, //3  Position FF  kS
    0, //4  Position FF  kV
    0, //5  Position FF  kA
    0, //6  Reverse FF 0 = false or 1= true
    0, //7 Position FF Type
    //Options 
    //0 = normal forces alwasy opposses motion
    //1=constant in one derection motion elavator
    //2 = changing like a pivoit or arm

    0, //8  Position PID kP
    0, //9  Position PID kI
    0, //10  Position PID kD
    0, //11 Position FF  kS
    0, //12  Position FF  kV
    0, //13  Position FF  kA
  };

  public static final double[] MotionConstants = {
    0, //0 Cruise Velocity
    0, //1 Acceleration
    0, //2 Jerk
    0, //3 Motion kV
    0, //4 Motion kA
  };

  }

  public static class Thrifty_LaserCAN{

            public static int CAN_ID = 61; // defines ID
            public static Boolean ShortRange = true; //Defines rnage type
            public static int[] Observation_Area={//number are pixel of the sencor
                8, // x starting values 0-16 8 is center
                8, // y starting values 0-16 8 is center
                16, // x-range from 1-16
                16  // y-range from 1-16
            };

            public static int ObservationTimingBudget = 1;
            /* 0 20Ms
             * 1 33Ms
             * 2 50Ms
             * 3 100Ms
             */
            public static double Blocked_Distance = 7.5; //This define the blocked state in cm

  }

  public static class Pneumatics_Soleniod_Double{ 
        int pneumatic_Hub = 0;//The ID of the power distipution hub
        int pneumatic_Channel = 0;//The ID of which channel the wire are in on the PDH
  }

  public static class Pneumatics_Soleniod_Single{ 
    int pneumatic_Hub = 0;//The ID of the power distipution hub
    int pneumatic_Channel_On = 0;//The ID of which channel the wire are in on the PDH for on
    int pneumatic_Channel_Off = 0;//The ID of which channel the wire are in on the PDH for off
    }

  }