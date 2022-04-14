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

  double setpoint = 0;

  VictorSP frontLeftDrive = new VictorSP(Constants.MotorControllers.FRONT_LEFT_DRIVE);
  VictorSP frontRightDrive = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_DRIVE);
  VictorSP backRightDrive = new VictorSP(Constants.MotorControllers.BACK_RIGHT_DRIVE);
  VictorSP backLeftDrive = new VictorSP(Constants.MotorControllers.BACK_LEFT_DRIVE);

  VictorSP frontLeftSteer = new VictorSP(Constants.MotorControllers.FRONT_LEFT_STEER);
  VictorSP frontRightSteer = new VictorSP(Constants.MotorControllers.FRONT_RIGHT_STEER);
  VictorSP backRightSteer = new VictorSP(Constants.MotorControllers.BACK_RIGHT_STEER);
  VictorSP backLeftSteer = new VictorSP(Constants.MotorControllers.BACK_LEFT_STEER);

  public MotorControllerGroup driveGroup = new MotorControllerGroup(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

  Encoder leadEncoder = new Encoder(0, 1);
  Encoder FREncoder = new Encoder(2, 3);
  Encoder BREncoder = new Encoder(4, 5);
  Encoder BLEncoder = new Encoder(6, 7);
  Encoder[] followEncoders = {FREncoder, BREncoder, BLEncoder, leadEncoder};
  VictorSP[] followWheels = {frontRightSteer, backRightSteer, backLeftSteer, frontLeftSteer};

  PIDController wheelControl = new PIDController(Constants.SteerPID.P, Constants.SteerPID.I, Constants.SteerPID.D);
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    wheelControl.enableContinuousInput(0, 360);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(Double power) {
    wheelControl.setSetpoint(power);
  }

  public void wheelFollow() {
    SmartDashboard.putNumber("Follow", FREncoder.get());
    for(int x = 0; x < 4; x++){
      followWheels[x].set(wheelControl.calculate(toDegrees(followEncoders[x])));
    }
  }

  public void resetEncoders(){
    for(int x = 0; x < 4; x++){
      followEncoders[x].reset();
    }
    setpoint = 0;
  }

  public double toDegrees(Encoder encoder){
    double toReturn = encoder.get() / 420 * 360 % 360;
    if(toReturn < 0){
      toReturn += 360;
    }
    return toReturn;
  }
}
