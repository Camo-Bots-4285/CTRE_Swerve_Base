// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BiFunction;

import frc.robot.RobotContainer;
import frc.robot.util.SelfDriving.Pathfinder;


/** An example command that uses an example subsystem. */
public class ToPose extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   public Pose2d POSE;
   public ToPose(double x, double  y, double rot_degrees){
    POSE = new Pose2d(
    new Translation2d(x, y),
    new Rotation2d(Units.degreesToRadians(rot_degrees))
  );
   }
  private Command m_autonomousCommand;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autonomousCommand= Pathfinder.moveToPose(POSE,0);
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

        return false;  

    }   
}
