// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Swerve system constants
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double trackWidth = Units.inchesToMeters(17);
    public static final double maxSpeed = 3.0; // meters per second for a single module
    public static final double maxAngularSpd = 7.0; // meters per second for a single module
    public static final double driveGearRatio = 6.75; 
    public static final double angleGearRatio = 150.0/7.0; // the MK4i have 150:7 gear ratio this is different than MK4
    public static final double wheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameterMeters * Math.PI;

    // Swerve Kinematics
    // MAKE SURE EACH MODULE IS DEFINE CORRECTLY
    public static final SwerveDriveKinematics swerveKinematics = 
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // FRONTLEFT - Positive x positive Y
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // FRONTRIGHT - Positive x, neg Y
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // backLeft - neg x, pos y
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back right - neg x, neg y
    );

    // Angle motor PID Values
    public static final double angleP = 0.03; // THIS IS FOR COLSON WHEELS
    public static final double angleI = 0.0;
    public static final double angleD = 0.0;
    public static final double angleFF = 0.0;

    // Drive conversions
    public static final double drivePositionConversion = (Math.PI * wheelDiameterMeters) / driveGearRatio;
    public static final double driveVelocityConversion = drivePositionConversion / 60;

    // Angle Conversion 
    public static final double anglePositionConversion = 360.0 / angleGearRatio;

    // Swerve Modules
    public static final Rotation2d angleOffsetMod0 = Rotation2d.fromDegrees(346.81);
    public static final Rotation2d angleOffsetMod1 = Rotation2d.fromDegrees(98.70);
    public static final Rotation2d angleOffsetMod2 = Rotation2d.fromDegrees(314.82);
    public static final Rotation2d angleOffsetMod3 = Rotation2d.fromDegrees(339.43);

    // CAN ID's
    public static final int pigeonID = 10;
    public static final boolean invertGyro = false;
} 