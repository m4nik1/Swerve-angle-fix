// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngle;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  static CommandXboxController mainJoy = new CommandXboxController(0);
  static Joystick turnJoy = new Joystick(1);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Robot.driveTrain.setDefaultCommand(new ElmCitySwerveControl());
  }

  public static double getLeftX() {
    return mainJoy.getLeftX();
  }

  public static double getLeftY() {
    return mainJoy.getLeftY();
  }

  public static double getRightX() {
    return mainJoy.getRightX();
  }

  public static boolean getRightBumper() {
    return mainJoy.rightBumper().getAsBoolean();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // mainJoy.a().onTrue(new TurnToAngle());
  }


}
