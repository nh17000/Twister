// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DiverterConstants.DiverterState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.TransferConstants.TransferState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.diverter.Diverter;
import frc.robot.subsystems.diverter.DiverterIO;
import frc.robot.subsystems.diverter.DiverterIOReal;
import frc.robot.subsystems.diverter.DiverterIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOReal;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOReal;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.*;
import frc.robot.util.heroheist.HeldGamePieceManager;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Turret turret;
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Hood hood;
    private final Diverter diverter;

    private final AutoAim aimAssist;

    private SwerveDriveSimulation driveSimulation = null;
    public RobotVisualizer visualizer = null;
    private HeldGamePieceManager manager = null;
    private BooleanSupplier multiNotePossession = () -> false; // todo real implementation w/ sensors

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);
    private static final CommandXboxController opController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                turret = new Turret(new TurretIOReal(), drive::getChassisSpeeds);
                intake = new Intake(new IntakeIOReal());
                transfer = new Transfer(new TransferIOReal());
                shooter = new Shooter(new ShooterIOReal());
                hood = new Hood(new HoodIOReal());
                diverter = new Diverter(new DiverterIOReal());
                aimAssist = new AutoAim(drive::getPose);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                SimulatedArena.overrideInstance(new Arena2024Crescendo());

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive // ,
                        // new VisionIOPhotonVisionSim(
                        //         camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        // new VisionIOPhotonVisionSim(
                        //         camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                        );
                turret = new Turret(new TurretIOSim(), drive::getChassisSpeeds);
                intake = new Intake(new IntakeIOSim());
                transfer = new Transfer(new TransferIOSim());
                shooter = new Shooter(new ShooterIOSim());
                hood = new Hood(new HoodIOSim());
                diverter = new Diverter(new DiverterIOSim());
                aimAssist = new AutoAim(driveSimulation::getSimulatedDriveTrainPose);

                visualizer = new RobotVisualizer(
                        turret::getTurretAngleRads,
                        hood::getAngleRadsToHorizontal,
                        diverter::getPivotAngleRadsToHorizontal);

                manager = new HeldGamePieceManager(
                        intake::getRollerVelocity,
                        transfer::getAngularVelocityRadPerSec,
                        turret::getTurretAngleRads,
                        shooter::getAngularVelocityRadPerSec,
                        diverter::getRollerVelocity,
                        visualizer::getHoodTransform,
                        visualizer::getDiverterTransform,
                        driveSimulation);

                multiNotePossession = manager::holdingMultipleNotes;

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                turret = new Turret(new TurretIO() {}, drive::getChassisSpeeds);
                intake = new Intake(new IntakeIO() {});
                transfer = new Transfer(new TransferIO() {});
                shooter = new Shooter(new ShooterIO() {});
                hood = new Hood(new HoodIO() {});
                diverter = new Diverter(new DiverterIO() {});
                aimAssist = new AutoAim(drive::getPose);

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        if (visualizer == null) {
            visualizer = new RobotVisualizer(
                    turret::getTurretAngleRads,
                    hood::getAngleRadsToHorizontal,
                    diverter::getPivotAngleRadsToHorizontal);
        }

        // Configure the button bindings
        configureButtonBindings();

        DriverStation.silenceJoystickConnectionWarning(Robot.isSimulation());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                true));
        turret.setDefaultCommand(aimAssist.aimAtSpeaker(turret, hood));
        intake.setDefaultCommand(new RunCommand(() -> intake.setState(IntakeState.INTAKING), intake));

        // --- Driver Controls ---
        controller.leftBumper().whileTrue(aimAssist.ampAlign(drive));

        controller
                .rightBumper()
                .onTrue(new InstantCommand(() -> transfer.setState(TransferState.TRANSFERRING)))
                .onFalse(new InstantCommand(() -> transfer.setState(TransferState.OFF)));

        controller
                .leftTrigger()
                .onTrue(new InstantCommand(() -> diverter.setState(DiverterState.DEPLOYED)))
                .whileTrue(aimAssist.aimAtAmp(turret, hood))
                .onFalse(new InstantCommand(() -> diverter.setState(DiverterState.STOWED)));

        // prevent longer than momentary control of multiple notes
        Trigger dualWieldingTrigger = new Trigger(multiNotePossession).debounce(2.5);

        // eject one note
        controller
                .b()
                .or(dualWieldingTrigger)
                .whileTrue(new RunCommand(
                        () -> {
                            intake.setState(IntakeState.EJECTING);
                        },
                        intake));

        // eject both notes
        controller
                .x()
                .whileTrue(new RunCommand(
                        () -> {
                            intake.setState(IntakeState.EJECTING);
                            transfer.setState(TransferState.REVERSE);
                            diverter.setState(DiverterState.REVERSE);
                        },
                        intake))
                .onFalse(new InstantCommand(() -> {
                    transfer.setState(TransferState.OFF);
                    diverter.setState(DiverterState.STOWED);
                }));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // --- Operator Controls ---
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.setPose(new Pose2d(15.22, 5.516, Rotation2d.kCW_90deg));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        manager.periodic();
        Logger.recordOutput("FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
        Logger.recordOutput(
                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
