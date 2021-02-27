// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;
  private int countR = 0;
  private int countL = 0;

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param speed The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */


  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
 
 
    // THis is not checking being off
    SmartDashboard.putNumber("Left Drive Inches", m_drive.getLeftDistanceInch());
    SmartDashboard.putNumber("Right Drive Inches", m_drive.getRightDistanceInch());
    double right = m_drive.getRightDistanceInch();
    double left = m_drive.getLeftDistanceInch();
   
    // set distance beween encoders that we will react to by speeding up the slower side
    double distDiff = left-right;
    double distDiffMax = 0.1;

    SmartDashboard.putNumber("diff", distDiff);

    // left side ahead of right side -> speed up right
    if ((left-right)>distDiffMax){
      SmartDashboard.putNumber("Setting Right Inc", countR++); // publish number of times we call this code
      m_drive.setSpeedRight(-0.05);  // right side is a negative speed for forward movement
      
    }
    // right side ahead of left side -> speed up left
    else if ((right-left)>distDiffMax){
      SmartDashboard.putNumber("Setting Left Inc", countL++); // publish number of times we call this code
            m_drive.setSpeedLeft(0.05);
    }

    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
