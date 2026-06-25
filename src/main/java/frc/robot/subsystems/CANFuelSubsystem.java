// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX intakeLauncherFollower;
  private final TalonFX intakeLauncherLeader;
  private final SparkMax feeder;
  private final SparkClosedLoopController feederController;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherLeader = new TalonFX(FOLLOWER_MOTOR_ID);
    intakeLauncherFollower = new TalonFX(INTAKE_LAUNCHER_MOTOR_ID);
    feeder = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    feederController = feeder.getClosedLoopController();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.3)
        .i(0.0)
        .d(0.0)
        .feedForward
        .kV(1.2);

    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VELOCITY);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VELOCITY);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VELOCITY);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VELOCITY);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VELOCITY);

    // create theget configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();
    launcherConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    launcherConfig.CurrentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kV = 0.052;
    intakeLauncherLeader.getConfigurator().apply(launcherConfig);
    intakeLauncherFollower.getConfigurator().apply(launcherConfig);
    intakeLauncherLeader.getConfigurator().apply(slot0Configs);
    intakeLauncherFollower.getConfigurator().apply(slot0Configs);
    intakeLauncherFollower.setControl(new Follower(FOLLOWER_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  public void intake() {
    feeder.setVoltage(
        8 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VELOCITY));
    intakeLauncherLeader.setControl(
        new VelocityVoltage(
                SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VELOCITY))
            .withSlot(0));
  }
  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feeder.setVoltage(-6);
    intakeLauncherLeader.setControl(
        new VelocityVoltage(
                -SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VELOCITY))
            .withSlot(0));
    intakeLauncherFollower.setControl(new StrictFollower(FOLLOWER_MOTOR_ID));
    // intakeLauncherLeader.setControl(new VelocityVoltage(-1 * SmartDashboard.getNumber("Intaking
    // launcher roller value", INTAKING_INTAKE_VELOCITY)).withSlot(0));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feeder.setVoltage(
        -6.0
            * SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VELOCITY));
    intakeLauncherLeader.setControl(
        new VelocityVoltage(
                -SmartDashboard.getNumber(
                    "Launching launcher roller value", LAUNCHING_LAUNCHER_VELOCITY))
            .withSlot(0));
    intakeLauncherFollower.setControl(new StrictFollower(FOLLOWER_MOTOR_ID));
  }

  // A method to stop the rollers
  public void stop() {
    feeder.setVoltage(0);
    intakeLauncherLeader.set(0);
    intakeLauncherFollower.setControl(new StrictFollower(FOLLOWER_MOTOR_ID));
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    // setPoint = SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VELOCITY);
    // feederController
    //    .setSetpoint(, ),ControlType.kVelocity);
    feeder.setVoltage(
        -6 * SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VELOCITY));
    intakeLauncherLeader.setControl(
        new VelocityVoltage(
                SmartDashboard.getNumber(
                    "Spin-up launcher roller value", LAUNCHING_LAUNCHER_VELOCITY))
            .withSlot(0));
    intakeLauncherFollower.setControl(new StrictFollower(FOLLOWER_MOTOR_ID));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // feederController.setSetpoint(setPoint, ControlType.kVelocity);

    // This method will be called once per scheduler run
  }
}
