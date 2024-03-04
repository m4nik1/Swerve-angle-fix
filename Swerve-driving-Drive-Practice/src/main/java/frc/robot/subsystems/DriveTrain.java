// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// https://github.com/frc3512/SwerveBot-2022

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private static Pigeon2 gyro;

  // private SwerveDriveOdometry swerveOdom;
  private ElmSwerveModule[] elmSwerveMods;
  private SwerveDriveOdometry swerveDriveOdometry;
 
  private Field2d field;

  public DriveTrain() {
    gyro = new Pigeon2(Constants.pigeonID); // make sure to change this id
    zeroHeading();
    
    elmSwerveMods = new ElmSwerveModule[] {
      new ElmSwerveModule(0, 18, 19, false, 
                          true, 12, Constants.angleOffsetMod0),
      new ElmSwerveModule(1, 4, 3, false, 
                          true, 7, Constants.angleOffsetMod1),
      new ElmSwerveModule(2, 20, 17, true, 
                          true, 13, Constants.angleOffsetMod2),
      new ElmSwerveModule(3, 2, 1, true, 
                          true, 22, Constants.angleOffsetMod3),
    };

    swerveDriveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void stop() {
    elmSwerveMods[0].stopMod();
    elmSwerveMods[1].stopMod();
    elmSwerveMods[2].stopMod();
    elmSwerveMods[3].stopMod();
  }

  public void zeroHeading() {
    gyro.setYaw(0);
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(ElmSwerveModule mod : elmSwerveMods) {
      positions[mod.mod] = mod.getPosition(); 
    }

    return positions;
  }

  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());
    // ChassisSpeeds spds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation); // robot centric

    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

    for(ElmSwerveModule m : elmSwerveMods) {
      m.setDesiredState(moduleStates[m.mod]);
    }
  }

  public void setAngle() {
    for(ElmSwerveModule m: elmSwerveMods) {
      elmSwerveMods[m.mod].setAnglePos();
    }
  }

  public double turningPos() {
    return elmSwerveMods[0].getTurningPosition();
  }

  public Rotation2d getYaw() {
    return (Constants.invertGyro)
      ? gyro.getRotation2d()
      : gyro.getRotation2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDriveOdometry.update(getYaw(), getPositions()); // getting the swerve positions and angle of robot for odomtry

    SmartDashboard.putNumber("Robot Heading", getYaw().getDegrees());

    for (ElmSwerveModule m : elmSwerveMods) {
      SmartDashboard.putNumber("Swerve Integrated " + m.mod, m.getAngle().getDegrees());
      SmartDashboard.putNumber("Swerve Velocity " + m.mod, m.getDriveVelocity());
      SmartDashboard.putNumber("Swerve Drive " + m.mod, m.getDrivePosition());
      SmartDashboard.putNumber("Swerve CANCoder " + m.mod, m.getCoderOffset().getDegrees());
    }
  }
}
