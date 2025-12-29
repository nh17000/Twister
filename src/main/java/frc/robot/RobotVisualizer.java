package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.DiverterConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class RobotVisualizer {
    private final DoubleSupplier turretYawSupplier;
    private final DoubleSupplier hoodPitchSupplier;
    private final DoubleSupplier diverterPitchSupplier;

    @Getter
    private Transform3d hoodTransform = Transform3d.kZero;

    @Getter
    private Transform3d diverterTransform = Transform3d.kZero;

    public void periodic() {
        double turretYaw = turretYawSupplier.getAsDouble();
        double hoodPitch = hoodPitchSupplier.getAsDouble();
        double diverterPitch = diverterPitchSupplier.getAsDouble();

        Logger.recordOutput("RobotVisualizer/Origin", Pose3d.kZero);

        // test w/ Math.cos(RobotController.getFPGATime() / 1e6);

        Transform3d turretTransform = new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, -turretYaw));
        hoodTransform = turretTransform.plus(new Transform3d(
                VisualizerConstants.SHOOTER_ZERO, new Rotation3d(0, hoodPitch - HoodConstants.HOOD_STARTING_ANGLE, 0)));
        diverterTransform = turretTransform.plus(new Transform3d(
                VisualizerConstants.DIVERTER_ZERO,
                new Rotation3d(0, diverterPitch - DiverterConstants.PIVOT_STARTING_ANGLE, 0)));

        Logger.recordOutput(
                "RobotVisualizer/Components2", new Transform3d[] {turretTransform, hoodTransform, diverterTransform});
    }
}
