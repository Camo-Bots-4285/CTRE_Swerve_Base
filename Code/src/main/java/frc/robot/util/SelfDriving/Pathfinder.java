package frc.robot.util.SelfDriving;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.SwerveBase.*;

public class Pathfinder extends SubsystemBase {

    // Defines the new subsytem
    private static CommandSwerveDrivetrain m_drive;

    //Configures new subsytem
    public Pathfinder(CommandSwerveDrivetrain m_drive) {
        this.m_drive = m_drive;

        // Bring in the auto configure to allow autos to runn
        m_drive.configureAutoBuilder();
    }

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    static PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static Command moveToPose(Pose2d targetPose, double endVelocity) {
        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                endVelocity);
        return pathfindingCommand;
    }

    public static Command moveToPath(String PathName) throws FileVersionException, IOException, ParseException {

        // Load the path we want to pathfind to and follow
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

        return pathfindingCommand;

    }

    public static Command PathNotRecognized (String PathName){
        return new RunCommand(() -> System.out.println("Could not find" + PathName));
    }
public static boolean PathEnded = false;

    public static Command PathEnded(){
        return new RunCommand(() -> PathEnded=true);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Pathend", PathEnded);
    }

}
