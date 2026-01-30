package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    @AutoLogOutput
    @Getter
    @Setter
    private ShooterState state = ShooterState.ON;

    private final LoggedTunableNumber shooterVolts = new LoggedTunableNumber("Shooter/Volts", ShooterState.ON.volts);

    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        double setpointVolts = state == ShooterState.ON ? shooterVolts.get() : state.volts;

        io.runVolts(setpointVolts);

        Logger.recordOutput("Shooter/Setpoint Volts", setpointVolts);
    }

    @AutoLogOutput
    public double getAngularPositionRads() {
        return inputs.shooterOneData.position() * ShooterConstants.SHOOTER_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getAngularVelocityRadPerSec() {
        return inputs.shooterOneData.velocity() * ShooterConstants.SHOOTER_P_COEFFICIENT;
    }

    @AutoLogOutput
    public double getFuelExitVelocity() {
        return getAngularVelocityRadPerSec() * ShooterConstants.SHOOTER_RADIUS * ShooterConstants.EFFICIENCY;
    }
}
