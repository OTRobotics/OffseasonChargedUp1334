// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.OperatorInput;



public class DriveSubsystem extends SubsystemBase {

    // Left side motors
    private final CANSparkMax     leftPrimaryMotor   = new CANSparkMax(DriveConstants.LEFT_MOTOR,
        MotorType.kBrushless);

    private final CANSparkMax     leftFollowerMotor  = new CANSparkMax(DriveConstants.LEFT_MOTOR_FOLLOWER,
        MotorType.kBrushless);

    // Right side motors
    private final CANSparkMax     rightPrimaryMotor  = new CANSparkMax(DriveConstants.RIGHT_MOTOR,
        MotorType.kBrushless);

    private final CANSparkMax     rightFollowerMotor = new CANSparkMax(DriveConstants.RIGHT_MOTOR_FOLLOWER,
        MotorType.kBrushless);

    private double                leftSpeed          = 0;
    private double                rightSpeed         = 0;

    // Drive encoders
    private final RelativeEncoder leftEncoder        = leftPrimaryMotor.getEncoder();
    private final RelativeEncoder rightEncoder       = rightPrimaryMotor.getEncoder();

    // Gyro
    private AHRS                  ahrs               = new AHRS(SPI.Port.kMXP);


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        leftPrimaryMotor.follow(ExternalFollower.kFollowerDisabled, 0);
        rightPrimaryMotor.follow(ExternalFollower.kFollowerDisabled, 0);

        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        // Set the drive motors to brake
        leftPrimaryMotor.setIdleMode(IdleMode.kBrake);
        leftFollowerMotor.setIdleMode(IdleMode.kBrake);

        rightPrimaryMotor.setIdleMode(IdleMode.kBrake);
        rightFollowerMotor.setIdleMode(IdleMode.kBrake);
    }

    // FIXME: Methods should have a lower case letter: tankDrive()
    public void TankDrive(double leftSpeed, double rightSpeed) {

        // Drive the left and right sides of the neos
        leftPrimaryMotor.set(leftSpeed);
        leftFollowerMotor.set(leftSpeed);
        rightPrimaryMotor.set(rightSpeed);
        rightFollowerMotor.set(rightSpeed);
    }

    // FIXME: Methods should have a lower case letter: arcadeDrive()
    public void ArcadeDrive(double speed, double turn) {

        // FIXME: Is this method only called from the DefaultDriveCommand?
        //
        // I would move this whole method about ramping into the default
        // drive command since it is about joystick adjustments/filters.
        //
        // Commands are the interfaces between the OperatorInput (OI) and
        // the subsystems (motors and sensors).
        //
        // btw, I am not sure that the drive method belongs here either,
        // maybe you should be calling set(leftSpeed, rightSpeed) from
        // the DefaultDriveCommand, and the Tank/Arcade drive calculations
        // should be in that command as well.

        if (OperatorInput.getSpeedRamp()) { // here's the error
            speed = speedRamp(speed);
            turn  = speedRamp(turn);
        }

        TankDrive((speed + turn), (speed - turn));
    }

    private double speedRamp(double speed) {

        // NOTE: You can use Math.signum() to get the sign +/- 1 of a value
        // This is a fun formula - never seen this before.
        //
        // return Math.signum(speed) * (0.2 * Math.abs(Math.pow(speed, 3))
        // + 0.8 * Math.pow(speed, 2));

        if (speed > 0) {
            return 0.2 * (Math.pow(speed, 3)) + 0.8 * (Math.pow(speed, 2));
        }
        else {
            speed = speed * -1;
        }
        return -1 * (0.2 * (Math.pow(speed, 3)) + 0.8 * (Math.pow(speed, 2)));
    }

    public double getLeftEncoder() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoder() {
        return rightEncoder.getPosition();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceCm() {
        return getAverageEncoder() * DriveConstants.CM_PER_ENCODER_COUNT;
    }

    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    private double getPitch() {
        return -ahrs.getRoll();
    }

    private double getRoll() {
        return ahrs.getPitch();
    }

    private double getYaw() {
        return ahrs.getYaw();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // FIXME: Put stuff on the SmartDashboard.

    }

}
