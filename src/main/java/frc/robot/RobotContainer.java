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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.TransferConstants.TransferState;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAim.Goal;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
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
// import frc.robot.util.heroheist.HeroHeistArena;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
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
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Hood hood;

    public final RobotVisualizer visualizer;
    private final HeldGamePieceManager manager;

    private final AutoAlign align;
    private final AutoAim aimAssist;

    private SwerveDriveSimulation driveSimulation = null;

    private double z;

    // Controller
    private static final CommandXboxController controller = new CommandXboxController(0);

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
                spindexer = new Spindexer(new SpindexerIOReal());
                transfer = new Transfer(new TransferIOReal());
                shooter = new Shooter(new ShooterIOReal());
                hood = new Hood(new HoodIOReal());
                align = new AutoAlign(drive::getPose);
                aimAssist = new AutoAim(drive::getPose);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                var arena = new Arena2026Rebuilt();
                // arena.setEfficiencyMode(false);
                SimulatedArena.overrideInstance(arena);

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
                        drive,
                        new VisionIOFake(driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose)
                        // new VisionIOPhotonVisionSim(
                        //         camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                        );
                turret = new Turret(new TurretIOSim(), drive::getChassisSpeeds);
                intake = new Intake(new IntakeIOSim());
                spindexer = new Spindexer(new SpindexerIOSim());
                transfer = new Transfer(new TransferIOSim());
                shooter = new Shooter(new ShooterIOSim());
                hood = new Hood(new HoodIOSim());
                align = new AutoAlign(driveSimulation::getSimulatedDriveTrainPose);
                aimAssist = new AutoAim(driveSimulation::getSimulatedDriveTrainPose);

                DriverStationSim.setDsAttached(true);
                DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

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
                spindexer = new Spindexer(new SpindexerIO() {});
                transfer = new Transfer(new TransferIO() {});
                shooter = new Shooter(new ShooterIO() {});
                hood = new Hood(new HoodIO() {});
                align = new AutoAlign(drive::getPose);
                aimAssist = new AutoAim(drive::getPose);

                break;
        }

        registerNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("auto"));

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        visualizer = new RobotVisualizer(
                turret::getTurretAngleRads,
                transfer::getRoll,
                spindexer::getAngleRads,
                intake::getPivotAngleRadsToHorizontal,
                hood::getAngleRadsToHorizontal);

        manager = new HeldGamePieceManager(
                intake::getRollerVelocity,
                spindexer::getAngleRads,
                transfer::getAngularVelocityRadPerSec,
                shooter::getAngularVelocityRadPerSec,
                hood::getAngleRadsToHorizontal,
                turret::getTurretAngleRads,
                visualizer::getHoodTransform,
                driveSimulation);

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
        // turret.setDefaultCommand(aimAssist.aim(turret, hood));
        turret.setDefaultCommand(aimAssist.simpleAim(turret, () -> vision.getTargetX(0)));

        // --- Driver Controls ---
        controller.povLeft().whileTrue(align.reefAlignLeft(drive));
        controller.povDown().whileTrue(align.reefAlignMid(drive));
        controller.povRight().whileTrue(align.reefAlignRight(drive));

        controller
                .leftBumper()
                .onTrue(new InstantCommand(() -> intake.setState(IntakeState.DEPLOYED)))
                .onFalse(new InstantCommand(() -> intake.setState(IntakeState.STOWED)));

        controller
                .rightBumper()
                .onTrue(new InstantCommand(() -> transfer.setState(TransferState.TRANSFERRING)))
                .onFalse(new InstantCommand(() -> transfer.setState(TransferState.OFF)));

        controller
                .back()
                .onTrue(new InstantCommand(() -> {
                    intake.setState(IntakeState.EJECTING);
                    transfer.setState(TransferState.REVERSE);
                }))
                .onFalse(new InstantCommand(() -> {
                    intake.setState(IntakeState.STOWED);
                    transfer.setState(TransferState.OFF);
                }));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // --- Operator Controls ---
        controller.a().onTrue(new InstantCommand(() -> aimAssist.setCurrentGoal(Goal.HUB)));
        controller.x().onTrue(new InstantCommand(() -> aimAssist.setCurrentGoal(Goal.DOWNTOWN)));
        controller.b().onTrue(new InstantCommand(() -> aimAssist.setCurrentGoal(Goal.LOW_FOOTHILL)));
        controller.y().onTrue(new InstantCommand(() -> aimAssist.setCurrentGoal(Goal.HIGH_FOOTHILL)));
    }

    private void registerNamedCommands() {
        new EventTrigger("Deploy Intake").onTrue(new InstantCommand(() -> intake.setState(IntakeState.DEPLOYED)));
        new EventTrigger("Stow Intake").onTrue(new InstantCommand(() -> intake.setState(IntakeState.STOWED)));

        NamedCommands.registerCommand(
                "Shoot",
                new RunCommand(() -> transfer.setState(TransferState.TRANSFERRING))
                        .withTimeout(2)
                        .andThen(new InstantCommand(() -> transfer.setState(TransferState.OFF))));
        NamedCommands.registerCommand("Climb", new RunCommand(() -> this.z += 0.0067).withTimeout(1));
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

        this.z = 0;
        drive.setPose(new Pose2d(3.005, 2.881, Rotation2d.kCW_90deg));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        manager.periodic();
        Logger.recordOutput(
                "FieldSimulation/Pose",
                new Pose3d(driveSimulation.getSimulatedDriveTrainPose())
                        .transformBy(new Transform3d(0, 0, z, Rotation3d.kZero)));
        Logger.recordOutput(
                "FieldSimulation/Red Speech Bubbles",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        // Logger.recordOutput(
        //         "FieldSimulation/Blue Speech Bubbles",
        //         SimulatedArena.getInstance().getGamePiecesArrayByType("Blue Speech Bubble"));
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
