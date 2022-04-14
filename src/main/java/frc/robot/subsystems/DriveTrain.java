// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {

  VictorSP frontLeftDrive = new VictorSP(Constants.MotorControllers.FRONT_LEFT_DRIVE);
  VictorSP frontRightDrive = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_DRIVE);
  VictorSP backRightDrive = new VictorSP(Constants.MotorControllers.BACK_RIGHT_DRIVE);
  VictorSP backLeftDrive = new VictorSP(Constants.MotorControllers.BACK_LEFT_DRIVE);

  VictorSP frontLeftSteer = new VictorSP(Constants.MotorControllers.FRONT_LEFT_STEER);
  VictorSP frontRightSteer = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_STEER);
  VictorSP backRightSteer = new VictorSP(Constants.MotorControllers.BACK_RIGHT_STEER);
  VictorSP backLeftSteer = new VictorSP(Constants.MotorControllers.BACK_LEFT_STEER);

  public MotorControllerGroup driveGroup = new MotorControllerGroup(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

  Encoder FLEncoder = new Encoder(0, 1);
  Encoder FREncoder = new Encoder(2, 3);
  Encoder BREncoder = new Encoder(4, 5);
  Encoder BLEncoder = new Encoder(6, 7);

  Encoder[] steerEncoders = {FREncoder, BREncoder, BLEncoder, FLEncoder};
  VictorSP[] followWheels = {frontRightSteer, backRightSteer, backLeftSteer, frontLeftSteer};

  PIDController wheelControl = new PIDController(Constants.SteerPID.P, Constants.SteerPID.I, Constants.SteerPID.D);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // Setup PID loop
    wheelControl.enableContinuousInput(0, 359);
    wheelControl.setTolerance(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Stop wheel rotation
  public void noRotation() {
    for(int x = 0; x < 4; x++){
      followWheels[x].set(0.0);
    }
  }

  // Move wheels to given angle(s)
  public void wheelFollow(double angle, double[] offsets) {
    SmartDashboard.putNumber("Follow", FREncoder.get());
    for(int x = 0; x < 4; x++){
      // PID loop uses .calculate(measurement, setpoint)
      followWheels[x].set(wheelControl.calculate(toDegrees(steerEncoders[x]), angle + offsets[x]));
    }
  }

  public void resetEncoders(){
    for(int x = 0; x < 4; x++){
      steerEncoders[x].reset();
    }
  }

  public double toDegrees(Encoder encoder){
    return -((encoder.get() / Constants.Etcetera.STEER_RATIO * 360) % 360);
  }
}
