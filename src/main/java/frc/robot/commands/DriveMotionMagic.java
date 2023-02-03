// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm_Base_Test;

public class DriveMotionMagic extends CommandBase {
  Arm_Base_Test m_arm;
  int m_target;
  boolean m_done;

  /** Creates a new DriveMotionMagic. */
  public DriveMotionMagic(Arm_Base_Test arm, int target) {

    m_arm = arm;
    addRequirements(m_arm);
    m_target = target;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.configureMotionMagic(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_done = m_arm.driveMotionMagic(m_target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_done;
  }
}
