// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.DriveToDis;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  GenericHID controller = new GenericHID(Constants.OIConstants.kDriverControllerPort);
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> controller.getRawAxis(1), () -> controller.getRawAxis(2)));
    configureBindings();
  }

 
  private void configureBindings() {
    new JoystickButton(controller, 1).onTrue(new TurnToAngle(45, driveSubsystem));
    new JoystickButton(controller, 2).onTrue(new TurnToAngle(90, driveSubsystem));
    new JoystickButton(controller, 3).onTrue(new TurnToAngle(180, driveSubsystem));




  }

  public Command getAutonomousCommand() {
    return new DriveToDis(1, driveSubsystem).andThen(new TurnToAngle(90, driveSubsystem));
  }
}
