package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

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

    private Supplier<Pose2d> robotSupplier;
    private Supplier<ChassisSpeeds> speedsSupplier;

    public AutoAim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;
    }

    public Command aim(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get();
                    Pose2d targetPose = findDistrictTargetPose(robotPose);
                    turret.followTarget(() -> getTurretTarget(robotPose, targetPose));
                    hood.followTarget(() -> getHoodTargetAngle(robotPose, targetPose, currentGoal.height));
                },
                turret,
                hood);
    }

    private Pose2d findDistrictTargetPose(Pose2d robotPose) {
        return robotPose.nearest(currentGoal.locations);
    }

    private static Rotation2d getTurretTarget(Pose2d robotPose, Pose2d targetPose) {
        return robotPose
                .getRotation()
                .minus(robotPose.minus(targetPose).getTranslation().getAngle());
    }

    private double getHoodTargetAngle(Pose2d robotPose, Pose2d targetPose, double goalHeight) {
        double x = robotPose.minus(targetPose).getTranslation().getNorm();
        double h = goalHeight - ShooterConstants.EJECT_HEIGHT;
        double v = ShooterConstants.TANGENTIAL_VELOCITY_AT_12V; // try using the actual shooter speed

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
}
