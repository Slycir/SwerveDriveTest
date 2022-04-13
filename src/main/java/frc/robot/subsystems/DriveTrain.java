// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class DriveTrain extends SubsystemBase {

  VictorSP frontLeftDrive = new VictorSP(Constants.MotorControllers.FRONT_LEFT_DRIVE);
  VictorSP frontRightDrive = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_DRIVE);
  VictorSP backRightDrive = new VictorSP(Constants.MotorControllers.BACK_RIGHT_DRIVE);
  VictorSP backLeftDrive = new VictorSP(Constants.MotorControllers.BACK_LEFT_DRIVE);

  VictorSP frontLeftSteer = new VictorSP(Constants.MotorControllers.FRONT_LEFT_STEER);
  VictorSP frontRightSteer = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_STEER);
  VictorSP backRightSteer = new VictorSP(Constants.MotorControllers.BACK_RIGHT_STEER);
  VictorSP backLeftSteer = new VictorSP(Constants.MotorControllers.BACK_LEFT_STEER);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
