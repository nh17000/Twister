package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAim {
    private Supplier<Pose2d> robotSupplier;

    private final InterpolatingDoubleTreeMap pivotLerp = new InterpolatingDoubleTreeMap();

    private Pose2d targetPose = Pose2d.kZero;
    private Debouncer isAlignedDebouncer = new Debouncer(0.2);

    @Getter
    @AutoLogOutput
    private boolean isForwards = true;

    public AutoAim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotSupplier = robotPoseSupplier;

        pivotLerp.put(1.0, 63.5);
        pivotLerp.put(1.5, 55.0);
        pivotLerp.put(2.0, 48.0);
        pivotLerp.put(2.5, 42.0);
        pivotLerp.put(3.0, 38.0);
        pivotLerp.put(3.5, 34.0);
        pivotLerp.put(4.0, 32.25);
        pivotLerp.put(4.5, 30.5);
        pivotLerp.put(5.0, 30.0);
        pivotLerp.put(6.0, 27.75);
        pivotLerp.put(7.5, 26.25);
        pivotLerp.put(9.0, 25.25);
        pivotLerp.put(10.5, 24.75);
    }

    public Command aimAtAmp(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    turret.followTarget(() -> isForwards ? Rotation2d.k180deg : Rotation2d.kZero);
                    hood.followTarget(() -> HoodConstants.HOOD_AMPING_ANGLE);
                },
                turret,
                hood);
    }

    public Command aimAtSpeaker(Turret turret, Hood hood) {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = robotSupplier.get();
                    Pose2d targetPose = getSpeakerPose();
                    turret.followTarget(() -> getTurretTarget(robotPose, targetPose));
                    hood.followTarget(() -> getHoodTargetAngle(robotPose, targetPose));
                },
                turret,
                hood);
    }

    private Pose2d getSpeakerPose() {
        return RobotContainer.isRedAlliance() ? FieldConstants.RED_SPEAKER_POSE : FieldConstants.BLUE_SPEAKER_POSE;
    }

    private static Rotation2d getTurretTarget(Pose2d robotPose, Pose2d targetPose) {
        return robotPose
                .getRotation()
                .minus(robotPose.minus(targetPose).getTranslation().getAngle());
    }

    private double getHoodTargetAngle(Pose2d robotPose, Pose2d targetPose) {
        double x = robotPose.minus(targetPose).getTranslation().getNorm();
        double h = FieldConstants.SPEAKER_HEIGHT - ShooterConstants.EJECT_HEIGHT;
        // double v = ShooterConstants.TANGENTIAL_VELOCITY_AT_12V;
        // double discriminant = Math.pow(v, 4) - Constants.g * (Constants.g * Math.pow(x, 2) + 2 * h * Math.pow(v, 2));
        // double tanTheta = (Math.pow(v, 2) - Math.sqrt(discriminant)) / (Constants.g * x);
        // double launchAngle = Math.atan(tanTheta); // ~= Math.atan(h / x) w/ gravity compensation

        double launchAngle = Units.degreesToRadians(pivotLerp.get(x)); // Math.atan(h / x);

        Logger.recordOutput("AutoAim/x", x);
        Logger.recordOutput("AutoAim/atan degs", Units.radiansToDegrees(Math.atan(h / x)));
        Logger.recordOutput("AutoAim/launchAngle degs", Units.radiansToDegrees(launchAngle));

        return MathUtil.clamp(launchAngle, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
    }

    public Command ampAlign(Drive drive) {
        return new DriveToPose(drive, this::findAmpTargetPose, robotSupplier);
    }

    private Pose2d findAmpTargetPose() {
        Pose2d robotPose = robotSupplier.get();
        Rotation2d currRot2d = robotPose.getRotation();

        double fwdError = Rotation2d.kCCW_90deg.minus(currRot2d).getDegrees();
        double bwdError = Rotation2d.kCW_90deg.minus(currRot2d).getDegrees();

        isForwards = Math.abs(fwdError) < Math.abs(bwdError);

        Rotation2d targetRot = isForwards ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg;

        return targetPose = robotPose
                .nearest(FieldConstants.AMPS)
                .transformBy(new Transform2d(AlignConstants.AMP_OFFSET, targetRot));
    }

    @AutoLogOutput
    public boolean isAligned() {
        Transform2d error = targetPose.minus(robotSupplier.get());

        return error.getTranslation().getNorm() < AlignConstants.ALIGN_TRANSLATION_TOLERANCE
                && error.getRotation().getRadians() < AlignConstants.ALIGN_ROT_TOLERANCE;
    }

    @AutoLogOutput
    public boolean isAlignedDebounced() {
        return isAlignedDebouncer.calculate(isAligned());
    }
}
