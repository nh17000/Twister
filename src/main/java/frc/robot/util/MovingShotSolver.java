package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public class MovingShotSolver {

    private static final double g = 9.81;

    public static class ShotSolution {

        public final double timeOfFlight; // seconds
        public final double shooterSpeed; // m/s
        public final Rotation2d turretAngle; // field-relative turret angle

        public ShotSolution(double time, double speed, Rotation2d turretAngle) {
            this.timeOfFlight = time;
            this.shooterSpeed = speed;
            this.turretAngle = turretAngle;
        }
    }

    public static ShotSolution solve(
            Supplier<Pose2d> poseSupplier,
            ChassisSpeeds robotSpeeds,
            double goalX,
            double goalY,
            double goalHeight,
            double shooterHeight,
            double hoodAngle) {

        double Dx = goalX - poseSupplier.get().getX();
        double Dy = goalY - poseSupplier.get().getY();
        double Dz = goalHeight - shooterHeight;

        double robotVx = robotSpeeds.vxMetersPerSecond;
        double robotVy = robotSpeeds.vyMetersPerSecond;

        double t = 1.0; // Initial guess of ToF

        for (int i = 0; i < 25; i++) {

            double vxLaunch = (Dx / t) - robotVx;
            double vyLaunch = (Dy / t) - robotVy;

            double horizontalSpeed = Math.hypot(vxLaunch, vyLaunch);

            double vzLaunch = horizontalSpeed * Math.tan(hoodAngle);

            double f = (vzLaunch * t - 0.5 * g * t * t) - Dz; // h = vt - 1/2at^2

            // Numerical derivative for Newton's method (look up formula online for clarification)

            double dt = 1e-4;
            double t2 = t + dt;

            double vx2 = (Dx / t2) - robotVx;
            double vy2 = (Dy / t2) - robotVy;
            double h2 = Math.hypot(vx2, vy2);
            double vz2 = h2 * Math.tan(hoodAngle);

            double f2 = (vz2 * t2 - 0.5 * g * t2 * t2) - Dz;

            double fPrime = (f2 - f) / dt;

            t = t - f / fPrime;

            // set min bound for t
            if (t < 0.1) t = 0.01;
        }

        // Compute final launch velocity components

        double vxLaunch = (Dx / t) - robotVx;
        double vyLaunch = (Dy / t) - robotVy;

        double horizontalSpeed = Math.hypot(vxLaunch, vyLaunch);
        double vzLaunch = horizontalSpeed * Math.tan(hoodAngle);

        // Shooter wheel speed magnitude:

        double shooterSpeed = Math.sqrt(horizontalSpeed * horizontalSpeed + vzLaunch * vzLaunch);

        // Compute field-relative turret angle

        Rotation2d turretAngle = new Rotation2d(Math.atan2(vyLaunch, vxLaunch));

        return new ShotSolution(t, shooterSpeed, turretAngle);
    }
}
