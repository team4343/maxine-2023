// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveDrive.SwerveSubsystem;

public class DriveFieldRelative extends CommandBase {
  SwerveSubsystem drivetrain;
  XboxController controller;

  /** Creates a new operatorDriveCommand. */
  public DriveFieldRelative(SwerveSubsystem drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var leftX = (Math.abs(controller.getLeftX()) <= 0.05) ? 0 : controller.getLeftX();
    var leftY = (Math.abs(controller.getLeftY()) <= 0.05) ? 0 : controller.getLeftY();
    var rightX = (Math.abs(controller.getRightX()) <= 0.05) ? 0 : controller.getRightX();
    drivetrain.driveFieldRelative(leftX, leftY, rightX);
    SmartDashboard.putNumber("Controller LX", controller.getLeftX());
    SmartDashboard.putNumber("Controller LY",- controller.getLeftY());
    SmartDashboard.putNumber("Controller RX", controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
