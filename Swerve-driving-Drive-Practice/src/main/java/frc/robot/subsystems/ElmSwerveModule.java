package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.math.OnboardModuleState;

public class ElmSwerveModule {
    CANSparkMax driveMotor;
    CANSparkMax turnMotor;
    Rotation2d angleOffset;
    Rotation2d lastAngle = Rotation2d.fromDegrees(0);

    RelativeEncoder driveEncoder;
    RelativeEncoder integratedTurnEncoder;

    SparkPIDController driveController; // Not using the drive controller since we dont need it yet
    SparkPIDController turnRelativeController;
    CANcoder angleEncoder; // to keep track of the position of the turning motor
    int mod;

    public ElmSwerveModule(int modNum, int driveMotorID, int turnMotorID, boolean driveReversed, boolean turningReversed, int CANCoderID, Rotation2d angleOffset) {
        mod = modNum;

        // Constants
        this.angleOffset = angleOffset;

        // Drive Motor Config
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(driveReversed);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0); // resets
        driveEncoder.setPositionConversionFactor(Constants.drivePositionConversion);
        driveEncoder.setVelocityConversionFactor(Constants.driveVelocityConversion);
        driveController = driveMotor.getPIDController();
        driveMotor.burnFlash();
        driveController.setP(.7);
        driveController.setI(0.001);
        driveController.setD(1.2);
        driveController.setFF(0);

        // Turn Motor config
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setInverted(turningReversed);
        turnMotor.setIdleMode(IdleMode.kCoast);
        integratedTurnEncoder = turnMotor.getEncoder();
        integratedTurnEncoder.setPositionConversionFactor(Constants.anglePositionConversion); // 360.0 / angleGearRatio
        turnRelativeController = turnMotor.getPIDController();
        turnRelativeController.setP(Constants.angleP, 0);
        turnRelativeController.setI(0, 0);
        turnRelativeController.setD(0, 0);
        turnRelativeController.setFF(0, 0);
        turnMotor.burnFlash();


        // Angle CANCoder config
        angleEncoder = new CANcoder(CANCoderID);
        resetToAbsolute(); // integrated encoder is the same value as absolute encoder on startup
        

        // getting the last angle
        lastAngle = getState().angle;
    }

    public void resetToAbsolute() { 
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedTurnEncoder.setPosition(absolutePosition); // sets the position to zero for now
        turnRelativeController.setReference(absolutePosition, ControlType.kPosition);
    }

    public void setAnglePos() {
        turnRelativeController.setReference(0, ControlType.kPosition);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return integratedTurnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void setDesiredState(SwerveModuleState state) {
        state = OnboardModuleState.optimize(state, getState().angle);

        // open loop is true for this project only
        setAngle(state);
        // setAnglePos();
        setSpeed(state, true);
    }

    public void setAngle(SwerveModuleState state) {
        Rotation2d angle;
        double absoluteSpeed = Math.abs(state.speedMetersPerSecond);
        // prevents jittering if speed is calculated to be less than 1% :)
        if(absoluteSpeed <= (Constants.maxSpeed *.01)) {
            angle = lastAngle;
        }
        else {
            angle = state.angle;
        }

        // setting the controller to a position since it is tied to the turn motor
        turnRelativeController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean openLoop) {
        if(openLoop) { // if openLoop/teleop just scale the output by the max speed
            // set velocity for a constant and then tune the robot while its in the air
            // then tune the translation on the ground 
            double speedOutput = desiredState.speedMetersPerSecond / Constants.maxSpeed; // this is for each module
            // driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0);
            driveMotor.set(speedOutput);
        }
        // for now hope that openLoop isn't false
        // else {
            // driveController.setReference(desiredState.speedMetersPerSecond,
            //                              ControlType.kVelocity,
            //                             0,
            //                             feedforward.calcu)
        // }
    }

    

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            Rotation2d.fromDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees())
            // CANCoder config angleEncoder.getAbsolutePosition().getValueAsDouble()*360.0
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    public void stopMod() {
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public Rotation2d getAngle() {
        // CHANGE THIS TO CANCODER AFTER TUNING THE ANGLEOFFSETS IN TUNER X
        return Rotation2d.fromDegrees(integratedTurnEncoder.getPosition());
    }

    public void runAngle(double speed) {
        turnMotor.set(speed);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition().getValueAsDouble()*360));
    }

    public Rotation2d getCoderOffset() {
        return Rotation2d.fromDegrees((angleEncoder.getAbsolutePosition().getValueAsDouble()*360)-angleOffset.getDegrees());
    }
}