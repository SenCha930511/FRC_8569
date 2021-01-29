// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {

  private Solenoid ss = new Solenoid(0);

  public PneumaticsSubsystem() {

  }

  @Override
  public void periodic() {
    
  }

  public void forward(){
    ss.set(true);
  }
  
  public void reverse(){
    ss.set(false);
  }
  
}
