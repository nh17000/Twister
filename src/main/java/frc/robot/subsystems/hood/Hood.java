package frc.robot.subsystems.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public void followTarget(DoubleSupplier hoodTargetAngleSupplier) {
        double setpointRads = hoodTargetAngleSupplier.getAsDouble();
        double setpointRots = (setpointRads - HoodConstants.HOOD_STARTING_ANGLE) / HoodConstants.HOOD_P_COEFFICIENT;

        io.runPosition(setpointRots);

        Logger.recordOutput("Hood/Setpoint Rads from Horizontal", setpointRads);
        Logger.recordOutput("Hood/Setpoint Degs from Horizontal", Units.radiansToDegrees(setpointRads));
        Logger.recordOutput("Hood/Setpoint Rots", setpointRots);
    }

    @AutoLogOutput
    public double getAngleRadsToHorizontal() {
        return inputs.hoodData.position() * HoodConstants.HOOD_P_COEFFICIENT + HoodConstants.HOOD_STARTING_ANGLE;
    }
}
