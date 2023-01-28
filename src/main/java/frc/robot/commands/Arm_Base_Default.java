// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm_Base_Test;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Arm_Base_Default extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm_Base_Test m_subsystem;
  private final XboxController m_controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_arm The subsystem used by this command.
   */
  public Arm_Base_Default(Arm_Base_Test m_arm, XboxController controller) {
    m_subsystem = m_arm;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = -m_controller.getRawAxis(1); // Left Y

    m_subsystem.drive_arm_manually(drive);
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
