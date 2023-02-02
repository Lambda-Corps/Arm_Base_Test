// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm_Base_Test extends SubsystemBase {
  private final WPI_TalonFX m_base_motor;
  
  private final int ARM_BOTTOM_STAGE = 6;
  private final int ARM_BOTTOM_REVERSE_LIMIT = 32000;
  private final int kGearRatio = 10 * 7 * 7;
  private final double ARM_BOTTOM_MAX_DUTY_CYCLE = .75; // Start with 20% max
  private final double ARM_BOTTOM_MAX_CURRENT = 5.0; // Start with 5 amps max total
  private final double Arm_BOTTOM_MAX_CURRENT_STATOR = 10.0; // Supplies 10 amps to the motors

  // Ticks per rotation in motor, * ratio, * 1/6 = 1/6 of the arm radial rotation
  // as the forward maximum for testing
  private final int ARM_BOTTOM_FORWARD_LIMIT = 2048 * kGearRatio * 1/6;

  /** Creates a new ExampleSubsystem. */
  public Arm_Base_Test() {
    m_base_motor = new WPI_TalonFX(ARM_BOTTOM_STAGE);

    TalonFXConfiguration base_config = new TalonFXConfiguration();

    // Factory default the talon
    m_base_motor.configFactoryDefault();

    // Set the integrated sensor as the primary feedback
    m_base_motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    m_base_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    m_base_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    // Configure the limits
    base_config.reverseSoftLimitThreshold = ARM_BOTTOM_REVERSE_LIMIT;
    base_config.forwardSoftLimitThreshold = ARM_BOTTOM_FORWARD_LIMIT;
    base_config.forwardSoftLimitEnable = true;
    base_config.reverseSoftLimitEnable = true;
    base_config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    base_config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
    StatorCurrentLimitConfiguration stator_limits = new StatorCurrentLimitConfiguration(true, Arm_BOTTOM_MAX_CURRENT_STATOR, Arm_BOTTOM_MAX_CURRENT_STATOR + 1, 0);
    SupplyCurrentLimitConfiguration supply_limits = new SupplyCurrentLimitConfiguration(true, ARM_BOTTOM_MAX_CURRENT, ARM_BOTTOM_MAX_CURRENT + 1, 0);
    base_config.statorCurrLimit = stator_limits;
    base_config.supplyCurrLimit = supply_limits;
    base_config.peakOutputForward = ARM_BOTTOM_MAX_DUTY_CYCLE;
    base_config.peakOutputReverse = -ARM_BOTTOM_MAX_DUTY_CYCLE;

    // Configure the talon
    m_base_motor.configAllSettings(base_config);
    m_base_motor.setInverted(TalonFXInvertType.Clockwise);

    m_base_motor.setSelectedSensorPosition(0);
  }

  public void drive_arm_manually(double input){
    // Make sure we don't drive above our limits
    input = MathUtil.clamp(input, (-ARM_BOTTOM_MAX_DUTY_CYCLE / 3), ARM_BOTTOM_MAX_DUTY_CYCLE);

    m_base_motor.set(ControlMode.PercentOutput, input);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
