


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.IOException;
import java.util.function.BiFunction;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.util.FileVersionException;

import frc.robot.RobotContainer;
import frc.robot.util.SelfDriving.Pathfinder;


/** An example command that uses an example subsystem. */
public class ToPath extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

    public String PathName=null;
    public ToPath(String PathName) {
    this.PathName=PathName;
  }

   private Command m_autonomousCommand;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pathfinder.PathEnded=false;

    try {
        m_autonomousCommand = Pathfinder.moveToPath(PathName);
    } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        m_autonomousCommand=Pathfinder.PathNotRecognized(PathName);
    }
    
    m_autonomousCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autonomousCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Add end command so if the path has come to the end pose the robot will not run it agian
    if (Pathfinder.PathEnded==true){
        return true;
    }
        return false;  

    }   
}
