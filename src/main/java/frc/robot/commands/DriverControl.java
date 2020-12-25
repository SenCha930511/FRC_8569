/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriverControl extends CommandBase {

  private double leftStickX;
  private double leftStickY;

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
