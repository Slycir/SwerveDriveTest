// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class DriveRobot extends CommandBase {

  DriveTrain m_driveTrain;
  DoubleSupplier m_moveX;
  DoubleSupplier m_moveY;

  double angle = 0.0;
  
  /** Creates a new DriveRobot. */
  public DriveRobot(DriveTrain driveTrain, DoubleSupplier moveX, DoubleSupplier moveY) {

    addRequirements(driveTrain);

    m_driveTrain = driveTrain;
    m_moveX = moveX;
    m_moveY = moveY;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Power", m_moveX.getAsDouble());
    // Get power (Squared by nature of pathagoreans theorum)
    double power = Math.pow(m_moveX.getAsDouble(), 2) + Math.pow(m_moveY.getAsDouble(), 2);
    // Offsets will be used in "Crab with a twist"
    double[] placeholderOffsets = {0,0,0,0};
    // 360 Degree deadband (think circle rather than cross)
    if(power < .15){m_driveTrain.driveGroup.set(0.0);m_driveTrain.noRotation();} else {

      // Handle straight up and down
      if(m_moveX.getAsDouble() == 0.0){
        if(m_moveY.getAsDouble() < 0.0){
          angle = (180.0);
        } else {
          angle = (0.0);
        }

      // Handle all other inputs
      // Stick to the right
      } else if(m_moveX.getAsDouble() > 0){
        angle = (90 - getExpectedWheelAngle());
        // System.out.println(angle);
      // Stick to the left
      } else if(m_moveX.getAsDouble() < 0){
        angle = (270 - getExpectedWheelAngle());
        // System.out.println(angle);
      }
      // Speed cap
      power = power * Constants.Etcetera.POWER_FACTOR;
      // m_driveTrain.drive(power);
      // m_driveTrain.driveGroup.set(power);
      // m_driveTrain.wheelFollow(angle, placeholderOffsets);

    }
    // Move wheels towards angles
    m_driveTrain.wheelFollow(angle, placeholderOffsets);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  double getExpectedWheelAngle() {
    return Math.atan(m_moveY.getAsDouble()/m_moveX.getAsDouble())*(180/Math.PI);
  }
}
