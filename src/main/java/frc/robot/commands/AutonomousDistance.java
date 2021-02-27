// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    double speed = 0.35;
    addCommands(
      new DriveDistance(speed, 16.5, drivetrain),
      new TurnDegrees(-1*speed, 85, drivetrain),
      new DriveDistance(speed, 19, drivetrain),
      new TurnDegrees(-1*speed, 115, drivetrain),
      new DriveDistance(speed, 28, drivetrain),
      new TurnDegrees(speed, 105, drivetrain),
      new DriveDistance(speed, 18, drivetrain),
      new TurnDegrees(speed, 85, drivetrain),
      new DriveDistance(speed, 18.5, drivetrain)
        );

  }
}
