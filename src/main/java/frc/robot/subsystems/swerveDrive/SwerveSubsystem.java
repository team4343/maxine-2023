// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.DriveConstants.swerveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private final AHRS gyro  = new AHRS(SPI.Port.kMXP);

  private final SwerveModule frontLeft = new SwerveModule(driveLeftFront, steerLeftFront, encoderLeftFront, thetaOffsetLeftFront, gyro, 1);
  private final SwerveModule frontRight = new SwerveModule(driveRightFront, steerRightFront, encoderRightFront, thetaOffsetRightFront, gyro, 2);
  private final SwerveModule backLeft = new SwerveModule(driveLeftBack, steerLeftBack, encoderLeftBack, thetaOffsetLeftBack, gyro,3);
  private final SwerveModule backRight = new SwerveModule(driveRightBack, steerRightBack, encoderRightBack, thetaOffsetRightBack, gyro,4);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(xyOffsetLeftFront, xyOffsetLeftBack, xyOffsetRightFront, xyOffsetRightBack);

  private void zeroGyro(){
    gyro.reset();
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    zeroGyro();
  }

  /*
   * Drive the robot given some field-relative x, y, and r values.
   */
  public void driveFieldRelative(double x, double y, double r) {
    // TODO: https://socratic.org/questions/if-you-have-10-8-meters-per-second-how-do-you-convert-that-to-radians-per-second
    //       use this to define rotations in m/s instead of rad/s.
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds((x * 4), (y * 4), (r * Math.PI * 2) * 1, Rotation2d.fromDegrees(gyro.getAngle())); 
    
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds); 
    
    SwerveModuleState frontLeftState = swerveModuleStates[0];
    SwerveModuleState frontRightState = swerveModuleStates[1];
    SwerveModuleState backLeftState = swerveModuleStates[2];
    SwerveModuleState backRightState = swerveModuleStates[3];

    frontLeft.setState(frontLeftState);
    frontRight.setState(frontRightState);
    backLeft.setState(backLeftState);
    backRight.setState(backRightState);
  }

  @Override
  public void periodic() {
    // TODO: update odometry based on swerve values. May not be necessary.
    //       Also consider where visual odometry will be going.
  }
}
