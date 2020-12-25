/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriverControl;

public class DriveTrain extends SubsystemBase {

  
  private PWMTalonSRX motorLeft_1 = new PWMTalonSRX(Constants.MOTOR_LEFT_1_ID);
  private PWMTalonSRX motorLeft_2 = new PWMTalonSRX(Constants.MOTOR_LEFT_2_ID);
  private PWMTalonSRX motorRight_1 = new PWMTalonSRX(Constants.MOTOR_RIGHT_1_ID);
  private PWMTalonSRX motorRight_2 = new PWMTalonSRX(Constants.MOTOR_RIGHT_2_ID);
  private SpeedControllerGroup motorLeft = new SpeedControllerGroup(motorLeft_1, motorLeft_2);
  private SpeedControllerGroup motorRight = new SpeedControllerGroup(motorRight_1, motorRight_2);
  //private SpeedController motorLeft = new PWMVictorSPX(Constants.MOTOR_LEFT);
  //private SpeedController motorRight = new PWMVictorSPX(Constants.MOTOR_RIGHT);
  private DifferentialDrive drive = new DifferentialDrive(motorLeft, motorRight);
  

  public DriveTrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void manualDrive(double move, double turn){
    if (Math.abs(move) < 0.10){
      move = 0;
    }
    if (Math.abs(turn) < 0.10){
      turn = 0;
    }
    drive.arcadeDrive(move * Constants.SPEED_SCALE, turn * Constants.SPEED_SCALE);
    SmartDashboard.putNumber("Robot_Speed", move);
  }


  public void setLeftMotors(double speed){
    //motorLeft_1.setInverted(true); //設置數值相反
    //motorLeft_2.setInverted(true);
    //motorLeft_1.set(ControlMode.PercentOutput, speed);
    //motorLeft_2.set(ControlMode.PercentOutput, speed);
    motorLeft.setInverted(true);
    motorLeft.set(speed);
  }


  public void setRightMotors(double speed){
    //motorRight_1.set(ControlMode.PercentOutput, speed);
    //motorRight_2.set(ControlMode.PercentOutput, speed);
    motorRight.set(speed);
  }


}
