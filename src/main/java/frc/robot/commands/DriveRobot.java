// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class DriveRobot extends CommandBase {

  DriveTrain m_driveTrain;
  DoubleSupplier m_moveX;
  DoubleSupplier m_moveY;
  
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
    double power = Math.pow(m_moveX.getAsDouble(), 2) + Math.pow(m_moveY.getAsDouble(), 2);
    if(power < .15){} else {
      if(m_moveX.getAsDouble() == 0.0){
        if(m_moveY.getAsDouble() < 0.0){
          m_driveTrain.setPower(180.0);
        } else {
          m_driveTrain.setPower(0.0);
        }
      } else if(m_moveX.getAsDouble() > 0){
        m_driveTrain.setPower(90 - Math.atan(m_moveY.getAsDouble()/m_moveX.getAsDouble())*(180/Math.PI));
      } else if(m_moveX.getAsDouble() < 0){
        m_driveTrain.setPower(270 - Math.atan(m_moveY.getAsDouble()/m_moveX.getAsDouble())*(180/Math.PI));
      }
      power = power/2;
      m_driveTrain.driveGroup.set(power);
    }
    m_driveTrain.wheelFollow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
