package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisualizerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.MovingShotSolver;
import frc.robot.util.MovingShotSolver.ShotSolution;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAim {
    @RequiredArgsConstructor
    public enum Goal {
        UPTOWN(FieldConstants.UPTOWN_DISTRICTS, FieldConstants.UPTOWN_HEIGHT),
        DOWNTOWN(FieldConstants.DOWNTOWN_DISTRICTS, FieldConstants.DOWNTOWN_HEIGHT),
        LOW_FOOTHILL(FieldConstants.FOOTHILL_DISTRICTS, FieldConstants.LOW_FOOTHILL_HEIGHT),
        HIGH_FOOTHILL(FieldConstants.FOOTHILL_DISTRICTS, FieldConstants.HIGH_FOOTHILL_HEIGHT),
        HUB(FieldConstants.HUBS, FieldConstants.HUB_HEIGHT);

        public final List<Pose2d> locations;
        public final double height;
    }

    @AutoLogOutput
    @Getter
    @Setter
    private Goal currentGoal = Goal.HUB;

    @AutoLogOutput
    private Pose2d goalPose = Pose2d.kZero;

    private Supplier<Pose2d> robotSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;
    private DoubleSupplier fuelExitVelSupplier;

    private static final Transform2d turretTransform =
            new Transform2d(VisualizerConstants.Z0_ZERO.toTranslation2d(), Rotation2d.kZero);

    public AutoAim(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            DoubleSupplier fuelExitVelSupplier) {
        this.robotSupplier = robotPoseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.fuelExitVelSupplier = fuelExitVelSupplier;
    }

    public Command aim(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get().transformBy(turretTransform.inverse());
                    Pose2d targetPose = findDistrictTargetPose(robotPose);
                    turret.followTarget(() -> getTurretTarget(robotPose, targetPose));
                    hood.followTarget(() -> getHoodTargetAngle(
                            robotPose, targetPose, currentGoal.height, fuelExitVelSupplier.getAsDouble()));
                },
                turret,
                hood);
    }

    private Pose2d findDistrictTargetPose(Pose2d robotPose) {
        // return robotPose.nearest(currentGoal.locations);
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)
                ? FieldConstants.BLUE_HUB
                : FieldConstants.RED_HUB;
    }

    private static Rotation2d getTurretTarget(Pose2d robotPose, Pose2d targetPose) {
        Rotation2d turretRotation = robotPose
                .getRotation()
                .minus(robotPose.minus(targetPose).getTranslation().getAngle());

        return turretRotation;
    }

    private double getHoodTargetAngle(Pose2d robotPose, Pose2d targetPose, double goalHeight, double v) {
        double x = robotPose.minus(targetPose).getTranslation().getNorm();
        double h = goalHeight - ShooterConstants.EJECT_HEIGHT;
        // double v = ShooterConstants.TANGENTIAL_VELOCITY_AT_12V; // try using the actual Shooter speed

        // use positive solution to shoot high and arc down into goal
        // (ball reaches maxiumum then falls into goal, e.g., 2022, 2026)
        // use negative solution to shoot straight into goal (e.g., 2020, 2024)
        boolean shouldArc = currentGoal == Goal.HUB;
        int sign = shouldArc ? 1 : -1;

        double discriminant = Math.pow(v, 4) - Constants.g * (Constants.g * Math.pow(x, 2) + 2 * h * Math.pow(v, 2));
        double tanTheta = (Math.pow(v, 2) + (sign * Math.sqrt(discriminant))) / (Constants.g * x);
        double launchAngle = Math.atan(tanTheta); // ~= Math.atan(h / x) w/ gravity compensation

        return MathUtil.clamp(Math.PI / 2 - launchAngle, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    }

    public Command simpleAim(Turret turret, Supplier<Rotation2d> txSupplier) {
        return new RunCommand(() -> {
            turret.followTarget(
                    () -> Rotation2d.fromRadians(turret.getTurretAngleRads()).plus(txSupplier.get()));
        });
    }

    public Command shootOnTheMove(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get();
                    Pose2d targetPose = findDistrictTargetPose(robotPose);

                    ChassisSpeeds speeds = speedsSupplier.get();
                    Translation2d robotVel =
                            new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(1);

                    double latency = 0.15;
                    Pose2d lookAheadPose = new Pose2d(
                            robotPose.getX() + speeds.vxMetersPerSecond * latency,
                            robotPose.getY() + speeds.vyMetersPerSecond * latency,
                            robotPose
                                    .getRotation()
                                    .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * latency)));

                    // horizontal distance to target (no compensation yet)
                    Translation2d rawTargetVec = targetPose.getTranslation().minus(lookAheadPose.getTranslation());
                    double dist = rawTargetVec.getNorm();

                    double v = fuelExitVelSupplier.getAsDouble();

                    // rough estimate
                    double tof1 = dist / Math.max(v, 0.1);

                    Translation2d target1 = targetPose.getTranslation().minus(robotVel.times(tof1));

                    double refinedDist =
                            target1.minus(lookAheadPose.getTranslation()).getNorm();

                    double tof2 = refinedDist / Math.max(v, 0.1);

                    Translation2d compensatedTarget =
                            targetPose.getTranslation().minus(robotVel.times(tof2));

                    Pose2d compensatedTargetPose = new Pose2d(compensatedTarget, targetPose.getRotation());

                    goalPose = compensatedTargetPose;

                    // --- Normal ballistic aiming using compensated target ---
                    turret.followTarget(() -> getTurretTarget(robotPose, compensatedTargetPose));

                    hood.followTarget(
                            () -> getHoodTargetAngle(robotPose, compensatedTargetPose, currentGoal.height, v));

                    Logger.recordOutput("AutoAim/SOTM/tof", tof2);
                    Logger.recordOutput("AutoAim/SOTM/rawDist", dist);
                    Logger.recordOutput("AutoAim/SOTM/robotVel", robotVel.getNorm());
                    Logger.recordOutput("AutoAim/SOTM/target", compensatedTargetPose);
                    Logger.recordOutput("AutoAim/SOTM/lookahead", lookAheadPose);
                },
                turret,
                hood);
    }

    public Command noTurretSOTM(Hood hood, Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return new RunCommand(
                        () -> {
                            Pose2d robotPose = robotSupplier.get().plus(turretTransform);
                            Pose2d targetPose = findDistrictTargetPose(robotPose);

                            ChassisSpeeds speeds = speedsSupplier.get();
                            Translation2d robotVel =
                                    new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

                            double latency = 0.1;
                            Pose2d lookAheadPose =
                                    robotPose.plus(new Transform2d(robotVel.times(-latency), Rotation2d.kZero));

                            // horizontal distance to target (no compensation yet)
                            Translation2d rawTargetVec =
                                    targetPose.getTranslation().minus(lookAheadPose.getTranslation());
                            double dist = rawTargetVec.getNorm();

                            double v = fuelExitVelSupplier.getAsDouble();

                            // rough estimate
                            double tof1 = dist / Math.max(v, 0.1);

                            Translation2d target1 = targetPose.getTranslation().minus(robotVel.times(tof1));

                            double refinedDist = target1.minus(lookAheadPose.getTranslation())
                                    .getNorm();

                            double tof2 = refinedDist / Math.max(v, 0.1);

                            Translation2d compensatedTarget =
                                    targetPose.getTranslation().minus(robotVel.times(tof2));

                            Pose2d compensatedTargetPose = new Pose2d(compensatedTarget, targetPose.getRotation());

                            goalPose = compensatedTargetPose;

                            // --- Normal ballistic aiming using compensated target ---
                            // turret.followTarget(() -> getTurretTarget(robotPose, compensatedTargetPose));

                            hood.followTarget(() ->
                                    getHoodTargetAngle(lookAheadPose, compensatedTargetPose, currentGoal.height, v));

                            Logger.recordOutput("AutoAim/SOTM/tof", tof2);
                            Logger.recordOutput("AutoAim/SOTM/rawDist", dist);
                            Logger.recordOutput("AutoAim/SOTM/robotVel", robotVel.getNorm());
                            Logger.recordOutput("AutoAim/SOTM/target", compensatedTargetPose);
                            Logger.recordOutput("AutoAim/SOTM/lookahead", lookAheadPose);
                        },
                        hood)
                .alongWith(DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> getAngle()));
    }

    public Command sotm2(Shooter shooter, Turret turret, DoubleSupplier hoodAngleSupplier) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get();
                    goalPose = findDistrictTargetPose(robotPose);
                    ShotSolution sol = MovingShotSolver.solve(
                            robotSupplier,
                            speedsSupplier.get(),
                            goalPose.getX(),
                            goalPose.getY(),
                            Goal.HUB.height,
                            ShooterConstants.EJECT_HEIGHT,
                            hoodAngleSupplier.getAsDouble());

                    shooter.setEject(sol.shooterSpeed);
                    turret.followTarget(
                            () -> robotPose.getRotation().minus(sol.turretAngle).plus(Rotation2d.k180deg));
                },
                shooter,
                turret);
    }

    private Rotation2d getAngle() {
        return robotSupplier
                .get()
                .getTranslation()
                .minus(goalPose.getTranslation())
                .getAngle();
    }
}
