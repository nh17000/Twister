package frc.robot.subsystems.diverter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.DiverterConstants;

public class DiverterIOSim extends DiverterIOTalonFX {
    private final SingleJointedArmSim pivotPhysicsSim = new SingleJointedArmSim(
            DiverterConstants.PIVOT_MOTOR,
            DiverterConstants.PIVOT_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(DiverterConstants.PIVOT_LENGTH, DiverterConstants.PIVOT_MASS),
            DiverterConstants.PIVOT_LENGTH,
            DiverterConstants.PIVOT_MIN_ANGLE,
            DiverterConstants.PIVOT_MAX_ANGLE,
            true,
            DiverterConstants.PIVOT_STARTING_ANGLE);

    private final DCMotorSim rollerPhysicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DiverterConstants.ROLLER_MOTOR,
                    DiverterConstants.ROLLER_SHAFT_MOI,
                    DiverterConstants.ROLLER_GEAR_RATIO),
            DiverterConstants.ROLLER_MOTOR);

    private final TalonFXSimState pivotSimState;
    private final TalonFXSimState rollerSimState;

    public DiverterIOSim() {
        pivotSimState = pivotMotor.getSimState();
        rollerSimState = rollerMotor.getSimState();
    }

    @Override
    public void updateInputs(DiverterIOInputs inputs) {
        super.updateInputs(inputs);

        pivotSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
        rollerSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

        pivotPhysicsSim.setInputVoltage(pivotSimState.getMotorVoltage());
        rollerPhysicsSim.setInputVoltage(rollerSimState.getMotorVoltage());

        pivotPhysicsSim.update(Constants.LOOP_PERIOD);
        rollerPhysicsSim.update(Constants.LOOP_PERIOD);

        pivotSimState.setRawRotorPosition((pivotPhysicsSim.getAngleRads() - DiverterConstants.PIVOT_STARTING_ANGLE)
                / DiverterConstants.PIVOT_P_COEFFICIENT);
        rollerSimState.setRawRotorPosition(
                rollerPhysicsSim.getAngularPositionRad() / DiverterConstants.ROLLER_P_COEFFICIENT);

        pivotSimState.setRotorVelocity(pivotPhysicsSim.getVelocityRadPerSec() / DiverterConstants.PIVOT_P_COEFFICIENT);
        rollerSimState.setRotorVelocity(
                rollerPhysicsSim.getAngularVelocityRadPerSec() / DiverterConstants.ROLLER_P_COEFFICIENT);
    }
}
