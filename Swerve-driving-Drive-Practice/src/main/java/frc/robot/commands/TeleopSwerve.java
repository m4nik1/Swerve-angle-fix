// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  SlewRateLimiter translationLimiter;
  SlewRateLimiter strafeLimiter;
  SlewRateLimiter rotationLimiter;


  // WE ARE ALWAYS FIELD CENTRIC!
  // boolean isFieldRel = true;

  public TeleopSwerve() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);

    translationLimiter = new SlewRateLimiter(3.0);
    strafeLimiter = new SlewRateLimiter(3.0);
    rotationLimiter = new SlewRateLimiter(3.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speedMulti = .5; // 50% for now
    double forward = RobotContainer.getLeftY();
    double strafe = RobotContainer.getLeftX();
    // double strafe = 0;
    double rotation = RobotContainer.getRightX();
    // double rotation = 0;
    boolean turbo = RobotContainer.getRightBumper();

    if(turbo) {
      speedMulti = 0.8;
      SmartDashboard.putBoolean("Turbo", turbo);
    }
    else {
      speedMulti = 0.5;
      SmartDashboard.putBoolean("Turbo", turbo);
    }

    // apply deadbands and set a limit for speeds
    double translationVal = translationLimiter.calculate(speedMulti * MathUtil.applyDeadband(forward, .05));
    double strafeVal = strafeLimiter.calculate(speedMulti * MathUtil.applyDeadband(strafe, .05));
    double rotationVal = rotationLimiter.calculate(speedMulti * MathUtil.applyDeadband(rotation, .03));

    // getting X and Y into a translation2d
    Translation2d translation = new Translation2d(translationVal, strafeVal);

    // multiply the speeds by the maxSpeeds constants
    Robot.driveTrain.drive(translation.times(Constants.maxSpeed),
                           rotationVal * Constants.maxAngularSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Robot.driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
