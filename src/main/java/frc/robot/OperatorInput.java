// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;


public class OperatorInput extends SubsystemBase{

  private final XboxController driverController;
  private final XboxController operatorController;

  public OperatorInput(int driverControllerPort, int operatorControllerPort) {

      driverController   = new XboxController(driverControllerPort);
      operatorController = new XboxController(operatorControllerPort);

  }

  public void configureBindings(DriveSubsystem driveSubsystem) {

  }


  public double getSpeed() {

      if (Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.15) {
          return (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis());
      }
      return 0.0;
  }

  public double getTurn () {

      if (Math.abs(driverController.getRawAxis(0)) > 0.15) {
          return (driverController.getRawAxis(0));
      }
      return 0.0;
  }

  public boolean getSpeedRamp() {
      return !driverController.getBButton();
  }

}
