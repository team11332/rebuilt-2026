// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final CANFuelSubsystem ballSubsystem = CANFuelSubsystem.getInstance();

  // The driver's controller
  private final CommandXboxController Controller =
      new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private final CommandXboxController yhavController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // תיקון: מחקנו מכאן את configureBindings() כי drive עדיין לא אותחל!

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings - עכשיו זה בטוח להרצה כי drive כבר קיים בזיכרון!
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -Controller.getLeftY(),
            () -> -Controller.getLeftX(),
            () -> -Controller.getRightX()));

    // Lock to 0° when A button is held
    Controller.a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -Controller.getLeftY(),
                () -> -Controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    Controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    Controller.b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
                    drive)
                .ignoringDisable(true));

    // While the left bumper on operator controller is held, intake Fuel
    Controller.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));

    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    Controller.rightBumper()
        .whileTrue(
            ballSubsystem
                .spinUpCommand()
                .withTimeout(SPIN_UP_SECONDS)
                .andThen(ballSubsystem.launchCommand())
                .finallyDo(() -> ballSubsystem.stop()));

    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    Controller.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));

    yhavController
        .a()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.feederIntakeScalarCommand(0.9)));

    yhavController
        .povDown()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.feederEjectScalarCommand(0.9)));

    yhavController
        .x()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.rollerIntakeScalarCommand(0.9)));

    yhavController
        .povLeft()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.rollerEjectScalarCommand(0.9)));

    yhavController
        .y()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.feederIntakeScalarCommand(1.1)));

    yhavController
        .povUp()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.feederEjectScalarCommand(1.1)));

    yhavController
        .b()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.rollerIntakeScalarCommand(1.1)));

    yhavController
        .povRight()
        .onTrue(ballSubsystem.run(() -> ballSubsystem.rollerEjectScalarCommand(1.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
