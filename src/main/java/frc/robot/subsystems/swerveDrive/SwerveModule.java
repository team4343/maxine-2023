// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerveDrive;

import static frc.robot.constants.DriveConstants.swerveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.Math;

public class SwerveModule extends SubsystemBase {
  private final TalonFX driveMotor;
  private final CANSparkMax angleMotor;
  private final CANCoder absoluteEncoder;

  private final RelativeEncoder steerEncoder;

  private final SparkMaxPIDController angleController;

  private final double rotationOffset;
  private final double rotationOffsetRad;

  // TODO: do we need this?
  private final AHRS gyro;

  private final int id;

  public SwerveModule(int driveID, int steerID, int absoluteID, double rotationOffset, AHRS gyro, int id) {
    driveMotor = new TalonFX(driveID);
    angleMotor = new CANSparkMax(steerID, MotorType.kBrushless);
    absoluteEncoder = new CANCoder(absoluteID);
    steerEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();

    this.rotationOffset = rotationOffset;
    this.rotationOffsetRad = (rotationOffset / 360) * (Math.PI * 2);

    this.gyro = gyro;
    this.id = id;
    
    configDriveMotor();
    configAngleMotor();

    // converts position and velocity to radians and radians per sec respectivily
  }

  // TODO: where do we use/need the following 3 methods?

  public double getAbsoluteEncoder() {
    return (absoluteEncoder.getAbsolutePosition());
  }

  public double getAbsoluteEncoderRad() {
      return (absoluteEncoder.getAbsolutePosition() / 360) * (Math.PI * 2);
  }

  public void updateSteerEncoder(){
    var value = (rotationOffset - getAbsoluteEncoder()) % 360;

    if (value > 180){
      value = value - 360;
    }

    System.out.print(String.valueOf(id) + ": " + String.valueOf(value));
     
    SmartDashboard.putNumber("Positon of Actual Val " + String.valueOf(id)+": ", value);
    steerEncoder.setPosition(value);
  }

  private void configDriveMotor() {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.configFactoryDefault();
    driveMotor.config_kF(0, 0.1);
    driveMotor.config_kP(0, 0.03); // Adjust PID constants as needed
    driveMotor.config_kI(0, 0.0);
    driveMotor.config_kD(0, 0.0);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    // TODO: do we need this?
    angleMotor.setInverted(false);
    angleMotor.setIdleMode(IdleMode.kBrake);
    // TODO: write an explanation for why we need this.
    steerEncoder.setPositionConversionFactor(360 / steerGearRatio);
    
    // TODO: do we need this?
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);
    angleController.setPositionPIDWrappingEnabled(true);

    // TODO: will this need more tuning?
    angleController.setP(0.01);
    angleController.setI(0);
    angleController.setD(0);
    angleController.setFF(0);

    // TODO: write an explanation for why we need this.
    updateSteerEncoder();
  }

  public void setAngle(double value){
    SmartDashboard.putNumber("Angle Final" + String.valueOf(id), value);

    angleMotor.getPIDController().setReference(value, ControlType.kPosition);
  }

  /**
   * Set the drive motor to run at a given speed, in meters per second.
   * 
   * TODO: verify if this is actually running at the real speeds.
   */
  public void setDrive(double linearSpeed ){
    double angularSpeed = linearSpeed / .0508;
    double sensorUnitsPerRotation = 2048 * 8.14;
    double sensorUnitsPer100Ms = (angularSpeed * sensorUnitsPerRotation / (2 * Math.PI)) / (10);
   
    driveMotor.set(ControlMode.Velocity, sensorUnitsPer100Ms);
  }

  // TODO: find the speedMetersPerSecond.
  public SwerveModuleState getState() {
    return new SwerveModuleState(0, new Rotation2d((getAbsoluteEncoderRad() - rotationOffsetRad) % (Math.PI * 2)));
  }

  public void setState(SwerveModuleState state){
    // TODO: check if steerEncoder.getPosition() is giving the right units.
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(steerEncoder.getPosition()));
    
    // TODO: find out if there's another alternative to this.
    //       This basically makes the steering coast instead of resetting back
    //       to zero. Saves some power.
    if (Math.abs(state.speedMetersPerSecond) < 0.1){
      angleMotor.set(0);
      setDrive(0);
    } else{
      setAngle(state.angle.getDegrees());
      setDrive(state.speedMetersPerSecond);
    }
  }

  @Override
  public void periodic() {
    // System.out.println(String.valueOf(id) + " - Absolute Encoder:   Deg:  " + String.valueOf(getAbsoluteEncoder()) + " --- Rad:  " + String.valueOf(getAbsoluteEncoderRad()));
  }
}
