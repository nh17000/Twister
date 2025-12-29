package frc.robot.util.heroheist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.DiverterConstants;
import frc.robot.Constants.DiverterConstants.DiverterState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

public class HeldGamePieceManager {
    private static final int TOTAL_CAPACITY = 2;
    // private static final Translation3d BUBBLE_TRANSLATION = new Translation3d(0.11, -0.336, 0.36);
    // private static final Rotation3d BUBBLE_ROT = new Rotation3d(0, Units.degreesToRadians(-120), 0);

    // private final LoggedTunableNumber x = new LoggedTunableNumber("Manager/x", 0);
    // private final LoggedTunableNumber y = new LoggedTunableNumber("Manager/y", 0);
    // private final LoggedTunableNumber z = new LoggedTunableNumber("Manager/z", 0.36);
    // private final LoggedTunableNumber roll = new LoggedTunableNumber("Manager/rol", 0);
    // private final LoggedTunableNumber pitch = new LoggedTunableNumber("Manager/pit", 60);
    // private final LoggedTunableNumber yaw = new LoggedTunableNumber("Manager/yaw", 0);
    // private final LoggedTunableNumber roll2 = new LoggedTunableNumber("Manager/rol2", 0);
    // private final LoggedTunableNumber pitch2 = new LoggedTunableNumber("Manager/pit2", 60);
    // private final LoggedTunableNumber yaw2 = new LoggedTunableNumber("Manager/yaw2", 0);

    private List<HeldNote> notes = new ArrayList<>();
    private boolean transferFull = false;

    private final DoubleSupplier intakeVelocitySupplier;
    private final DoubleSupplier transferVelocitySupplier;
    private final DoubleSupplier turretAngleSupplier;
    private final DoubleSupplier shooterVelocitySupplier;
    private final DoubleSupplier diverterVelocitySupplier;

    private final Supplier<Transform3d> hoodTransformSupplier;
    private final Supplier<Transform3d> diverterTransformSupplier;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final IntakeSimulation leftIntakeSim;
    private final IntakeSimulation rightIntakeSim;

    public HeldGamePieceManager(
            DoubleSupplier intakeVelocitySupplier,
            DoubleSupplier transferVelocitySupplier,
            DoubleSupplier turretAngleSupplier,
            DoubleSupplier shooterVelocitySupplier,
            DoubleSupplier diverterVelocitySupplier,
            Supplier<Transform3d> hoodTransformSupplier,
            Supplier<Transform3d> diverterTransformSupplier,
            AbstractDriveTrainSimulation driveSimulation) {

        this.intakeVelocitySupplier = intakeVelocitySupplier;
        this.transferVelocitySupplier = transferVelocitySupplier;
        this.turretAngleSupplier = turretAngleSupplier;
        this.shooterVelocitySupplier = shooterVelocitySupplier;
        this.diverterVelocitySupplier = diverterVelocitySupplier;

        this.hoodTransformSupplier = hoodTransformSupplier;
        this.diverterTransformSupplier = diverterTransformSupplier;

        this.poseSupplier = driveSimulation::getSimulatedDriveTrainPose;
        this.chassisSpeedsSupplier = driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative;

        leftIntakeSim = IntakeSimulation.InTheFrameIntake(
                "Note", driveSimulation, Inches.of(26), IntakeSimulation.IntakeSide.LEFT, 1);
        rightIntakeSim = IntakeSimulation.InTheFrameIntake(
                "Note", driveSimulation, Inches.of(26), IntakeSimulation.IntakeSide.RIGHT, 1);

        notes.add(new HeldNote(true)); // preload
    }

    public void periodic() {
        if (intakeVelocitySupplier.getAsDouble() > 0.1 && notes.size() < TOTAL_CAPACITY) {
            leftIntakeSim.startIntake();
            rightIntakeSim.startIntake();
        } else {
            leftIntakeSim.stopIntake();
            rightIntakeSim.stopIntake();
        }

        if (leftIntakeSim.obtainGamePieceFromIntake()) {
            notes.add(new HeldNote(true));
        }
        if (rightIntakeSim.obtainGamePieceFromIntake()) {
            notes.add(new HeldNote(false));
        }

        double intakeVel = intakeVelocitySupplier.getAsDouble() * IntakeConstants.ROLLER_RADIUS;
        double transferVel = transferVelocitySupplier.getAsDouble() * TransferConstants.TRANSFER_RADIUS;
        double diverterVel = diverterVelocitySupplier.getAsDouble() * DiverterConstants.ROLLER_RADIUS;
        double shooterVel =
                shooterVelocitySupplier.getAsDouble() * ShooterConstants.SHOOTER_RADIUS * ShooterConstants.EFFICIENCY;
        Pose2d robotPose = poseSupplier.get();

        boolean diverterDeployed = MathUtil.isNear(
                DiverterState.DEPLOYED.pivotRads,
                diverterTransformSupplier.get().getRotation().getY() + DiverterConstants.PIVOT_STARTING_ANGLE,
                Units.degreesToRadians(10));

        var iterator = notes.iterator();
        List<Pose3d> notePoses = new ArrayList<>();
        while (iterator.hasNext()) {
            HeldNote note = iterator.next();
            Location loc = note.update(
                    Constants.LOOP_PERIOD, intakeVel, transferVel, shooterVel, diverterVel, diverterDeployed);

            if (loc.equals(Location.EJECT_SHOOTER)) {
                iterator.remove();
                shoot(shooterVel);
            } else if (loc.equals(Location.EJECT_DIVERTER)) {
                iterator.remove();
                amp(diverterVel);
            } else if (loc.equals(Location.EJECT_LEFT_INTAKE)) {
                iterator.remove();
                intakeEject(intakeVel, true, robotPose);
            } else if (loc.equals(Location.EJECT_RIGHT_INTAKE)) {
                iterator.remove();
                intakeEject(intakeVel, false, robotPose);
            } else {
                notePoses.add(new Pose3d(robotPose).plus(note.transform));
            }
        }

        Logger.recordOutput("GamePieceManager/Held Notes", notePoses.toArray(new Pose3d[notePoses.size()]));
        Logger.recordOutput(
                "GamePieceManager/Test Bubble",
                new Pose3d(robotPose)
                        .transformBy(getNoteInShooterTransform(
                                2 + Math.pow(Math.cos(RobotController.getFPGATime() / 1e6), 2))));
        Logger.recordOutput(
                "GamePieceManager/Test Div",
                new Pose3d(robotPose)
                        .transformBy(getNoteInDiverterTransform(
                                3 + Math.pow(Math.sin(RobotController.getFPGATime() / 1e6), 2))));
    }

    public boolean holdingMultipleNotes() {
        return notes.size() > 1;
    }

    private void shoot(double shooterVel) {
        Transform3d shooterTransform = getNoteInShooterTransform(3);
        Translation2d shooterTranslation = shooterTransform.getTranslation().toTranslation2d();
        Rotation2d turretRotation = Rotation2d.fromRadians(-turretAngleSupplier.getAsDouble() - Math.PI);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new NoteOnFly(
                        poseSupplier.get().getTranslation(),
                        shooterTranslation.rotateBy(turretRotation.unaryMinus()),
                        chassisSpeedsSupplier.get(),
                        poseSupplier.get().getRotation().plus(turretRotation),
                        shooterTransform.getMeasureZ(),
                        MetersPerSecond.of(shooterVel),
                        shooterTransform.getRotation().getMeasureY()));
    }

    private void amp(double diverterVel) {
        Transform3d diverterTransform = getNoteInDiverterTransform(4);
        Translation2d diverterTranslation = diverterTransform.getTranslation().toTranslation2d();
        Rotation2d turretRotation = Rotation2d.fromRadians(-turretAngleSupplier.getAsDouble() - Math.PI);

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new NoteOnFly(
                        poseSupplier.get().getTranslation(),
                        diverterTranslation.rotateBy(turretRotation.unaryMinus()),
                        chassisSpeedsSupplier.get(),
                        poseSupplier.get().getRotation().plus(turretRotation),
                        diverterTransform.getMeasureZ(),
                        MetersPerSecond.of(diverterVel),
                        diverterTransform.getRotation().getMeasureY()));
    }

    private void intakeEject(double intakeVel, boolean isLeft, Pose2d robotPose) {
        SimulatedArena.getInstance()
                .addGamePiece(new CrescendoNoteOnField(robotPose
                        .getTranslation()
                        .plus(new Translation2d(0, 0.67 * (isLeft ? 1 : -1)).rotateBy(robotPose.getRotation()))));
    }

    private Transform3d getNoteInShooterTransform(double x) {
        double xTransform = -0.2 * (x - 2);
        return hoodTransformSupplier
                .get()
                .plus(new Transform3d(
                        new Translation3d(
                                        -VisualizerConstants.SHOOTER_ZERO.getX() + xTransform,
                                        -VisualizerConstants.SHOOTER_ZERO.getY(),
                                        -VisualizerConstants.SHOOTER_ZERO.getZ() + 0.36)
                                .rotateBy(new Rotation3d(0, Units.degreesToRadians(60), 0)),
                        new Rotation3d(0, Units.degreesToRadians(60), 0)));
        // return hoodTransformSupplier
        //         .get()
        //         .plus(new Transform3d(
        //                 new Translation3d(
        //                                 -VisualizerConstants.SHOOTER_ZERO.getX() + this.x.get() + xTransform,
        //                                 -VisualizerConstants.SHOOTER_ZERO.getY() + this.y.get(),
        //                                 -VisualizerConstants.SHOOTER_ZERO.getZ() + this.z.get())
        //                         .rotateBy(new Rotation3d(
        //                                 Units.degreesToRadians(this.roll.get()),
        //                                 Units.degreesToRadians(this.pitch.get()),
        //                                 Units.degreesToRadians(this.yaw.get()))),
        //                 new Rotation3d(
        //                         Units.degreesToRadians(this.roll2.get()),
        //                         Units.degreesToRadians(this.pitch2.get()),
        //                         Units.degreesToRadians(this.yaw2.get()))));
    }

    private Transform3d getNoteInDiverterTransform() {
        return diverterTransformSupplier
                .get()
                .plus(new Transform3d(
                        new Translation3d(
                                        -VisualizerConstants.DIVERTER_ZERO.getX() + -0.15,
                                        -VisualizerConstants.DIVERTER_ZERO.getY(),
                                        -VisualizerConstants.DIVERTER_ZERO.getZ() + 0.93)
                                .rotateBy(new Rotation3d(0, Units.degreesToRadians(-50), 0)),
                        new Rotation3d(0, Units.degreesToRadians(-50), 0)));

        // x += 1;
        // double pitch = Units.degreesToRadians(-(6 * x) * (x - 4));

        // return diverterTransformSupplier
        //         .get()
        //         .plus(new Transform3d(
        //                 new Translation3d(
        //                                 -VisualizerConstants.DIVERTER_ZERO.getX() + this.x.get(),
        //                                 -VisualizerConstants.DIVERTER_ZERO.getY() + this.y.get(),
        //                                 -VisualizerConstants.DIVERTER_ZERO.getZ() + this.z.get())
        //                         .rotateBy(new Rotation3d(
        //                                 Units.degreesToRadians(this.roll.get()),
        //                                 Units.degreesToRadians(this.pitch.get()),
        //                                 Units.degreesToRadians(this.yaw.get()))),
        //                 new Rotation3d(
        //                         Units.degreesToRadians(this.roll2.get()),
        //                         Units.degreesToRadians(this.pitch2.get()),
        //                         Units.degreesToRadians(this.yaw2.get()))));
    }

    private Transform3d getNoteInDiverterTransform(double x) {
        return Pose3d.kZero
                .transformBy(getNoteInShooterTransform(3))
                .interpolate(Pose3d.kZero.transformBy(getNoteInDiverterTransform()), x - 3)
                .minus(Pose3d.kZero);
    }

    class HeldNote {
        public double x;
        public Transform3d transform;
        public Location location;
        private boolean wasLeft;

        public HeldNote(boolean isLeft) {
            x = 0;
            transform = Transform3d.kZero;
            location = isLeft ? Location.LEFT_INTAKE : Location.RIGHT_INTAKE;
            wasLeft = isLeft;
        }

        public Location update(
                double dt, double intakeVel, double transferVel, double shooterVel, double diverterVel, boolean amp) {
            switch (this.location) {
                case LEFT_INTAKE -> {
                    x += intakeVel * dt;
                    transform = new Transform3d(0, 0.55 - 0.55 * x, 0.03 + 0.03 * x, Rotation3d.kZero);

                    if (x < -0.1) {
                        location = Location.EJECT_LEFT_INTAKE;
                    } else if (x > 1) {
                        if (transferFull) {
                            x = 0.9;
                        } else {
                            location = Location.TRANSFER;
                            x = 1;
                            transferFull = true;
                        }
                    }

                    break;
                }
                case RIGHT_INTAKE -> {
                    x += intakeVel * dt;
                    transform = new Transform3d(0, -0.55 + 0.55 * x, 0.03 + 0.03 * x, Rotation3d.kZero);

                    if (x < -0.1) {
                        location = Location.EJECT_RIGHT_INTAKE;
                    } else if (x > 1) {
                        if (transferFull) {
                            x = 0.9;
                        } else {
                            location = Location.TRANSFER;
                            x = 1;
                            transferFull = true;
                        }
                    }

                    break;
                }
                case TRANSFER -> {
                    x += transferVel * dt;
                    transform = getNoteInShooterTransform(2);

                    if (x > 2) {
                        location = Location.SHOOTER;
                        transferFull = false;
                        x = 2;
                    } else if (x < 1) {
                        location = wasLeft ? Location.LEFT_INTAKE : Location.RIGHT_INTAKE;
                        transferFull = false;
                        x = 0.9;
                    }

                    break;
                }
                case SHOOTER -> {
                    x += shooterVel * dt;
                    transform = getNoteInShooterTransform(x);

                    if (x > 3) {
                        location = amp ? Location.DIVERTER : Location.EJECT_SHOOTER;
                        x = 3;
                    } else if (x < 2) {
                        location = Location.TRANSFER;
                        transferFull = true;
                        x = 1.9;
                    }
                }
                case DIVERTER -> {
                    x += diverterVel * dt;
                    transform = getNoteInDiverterTransform(x);

                    if (x > 4) {
                        location = Location.EJECT_DIVERTER;
                    } else if (x < 3) {
                        location = Location.SHOOTER;
                    }
                }
                default -> {
                    return location;
                }
            }
            return location;
        }
    }

    public enum Location {
        LEFT_INTAKE, // 0-1
        RIGHT_INTAKE, // 0-1
        TRANSFER, // 1-2
        SHOOTER, // 2-3
        DIVERTER, // 3-4 (optional)

        EJECT_SHOOTER,
        EJECT_DIVERTER,
        EJECT_LEFT_INTAKE,
        EJECT_RIGHT_INTAKE
    }
}
