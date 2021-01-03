/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriverControl extends CommandBase {

  private double leftStickX;
  private double leftStickY;
  private double X_Axis;
  private double Y_Axis;
  private double Speed;

  public DriverControl() {
    addRequirements(RobotContainer.driveTrain);  //driveTrain控制DriverControl
  }

  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    leftStickX = Robot.robotContainer.GetJoystcikRawAxis(Constants.LEFT_STICK_X);
    leftStickY = Robot.robotContainer.GetJoystcikRawAxis(Constants.LEFT_STICK_Y);
    RobotContainer.driveTrain.manualDrive(leftStickY, leftStickX);
    /** 
    X_Axis = Robot.robotContainer.GetJoystcikRawAxis(Constants.X_AXIS);
    Y_Axis = Robot.robotContainer.GetJoystcikRawAxis(Constants.Y_AXIS);
    Speed = Robot.robotContainer.GetJoystcikRawAxis(Constants.SLIDER);
    RobotContainer.driveTrain.manualDrive(Y_Axis, X_Axis, Speed);
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveTrain.setLeftMotors(0);
    RobotContainer.driveTrain.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
