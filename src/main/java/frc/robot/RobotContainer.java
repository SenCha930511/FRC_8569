/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.commands.DriverControl;
import frc.robot.commands.Shooting;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Joystick joystick_1 = new Joystick(Constants.JOYSTICK_1_ID);
  private Joystick joystick_2 = new Joystick(Constants.JOYSTICK_2_ID);
  private Joystick joystick_3 = new Joystick(Constants.JOYSTICK_3_ID);
  private JoystickButton button_a  = new JoystickButton(joystick_1, Constants.BUTTON_A);
  private JoystickButton button_x  = new JoystickButton(joystick_1, Constants.BUTTON_X);
  private JoystickButton button_y  = new JoystickButton(joystick_1, Constants.BUTTON_Y);
  private JoystickButton button_1 = new JoystickButton(joystick_2, Constants.ARDUINO_BUTTON_1);
  public static final DriveTrain driveTrain = new DriveTrain();
  private final DriverControl drivercommand = new DriverControl();

  public double GetJoystcikRawAxis(int axis){
    return joystick_1.getRawAxis(axis);
  }

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    driveTrain.setDefaultCommand(drivercommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //button_1.whenHeld(new TestCommand());
    button_y.whenHeld(new Shooting());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /** 
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
