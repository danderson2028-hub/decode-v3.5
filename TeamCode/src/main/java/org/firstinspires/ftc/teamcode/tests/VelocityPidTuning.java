package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Velocity Tuning")
public class VelocityPidTuning extends NextFTCOpMode {
    public VelocityPidTuning(){
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    //GraphManager graphManager = PanelsGraph.INSTANCE.getManager();
    // ---- Dashboard-tunable PID coefficients ----
    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    // This object links live to the dashboard.
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);

    // ---- Tunable target velocity ----
    public static double targetVelocity = 0.0; // ticks per second
    public static double velocityStep = 50.0;

    // ---- Hardware ----
    private MotorEx intake;
    private MotorEx transfer;
    private MotorEx intake2;

    public static boolean runTransfer = false;

    // ---- Control System ----
    private ControlSystem controller;

    @Override
    public void onInit() {
        panelsTelemetry.addLine("Shooter PID Tuner Initialized");
        panelsTelemetry.update();

        // Initialize motor
        intake = new MotorEx("shooter");
        transfer = new MotorEx("transfer");
        intake2 = new MotorEx("intake");

        // Build controller with velocity PID using live-updating coefficients
        controller = ControlSystem.builder()
                .velPid(coefficients)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        panelsTelemetry.addLine("Shooter PID Tuner Started");
        panelsTelemetry.update();
    }

    @Override
    public void onUpdate() {
        if(runTransfer){
            intake2.setPower(-.7);
            transfer.setPower(.8);

        }
        else{
            intake2.setPower(0);
            transfer.setPower(0);
        }
        if(p!=coefficients.kP || i!=coefficients.kI || d!=coefficients.kD) {
            coefficients.kP = p;
            coefficients.kI = i;
            coefficients.kD = d;

            controller = ControlSystem.builder()
                    .velPid(coefficients)
                    .basicFF(f,0,0)
                    .build();
        }


        // Optional: only rebuild if values changed, or rebuild every loop for simplicity


        // === Handle Gamepad Input ===

        // === Control Loop ===
        double currentVelocity = intake.getVelocity(); // ticks per second
        double currentPosition = intake.getCurrentPosition();

        // Set goal (position not used here, but required by KineticState)
        controller.setGoal(new KineticState(currentPosition, targetVelocity));

        // Calculate motor power using current state
        double power = controller.calculate(new KineticState(currentPosition, currentVelocity));

        // Clamp power for safety
        power = Math.max(-1.0, Math.min(1.0, power));

        intake.setPower(power);


        // === Telemetry ===
        panelsTelemetry.addData("Target Velocity (ticks/s)", targetVelocity);
        panelsTelemetry.addData("Measured Velocity (ticks/s)", currentVelocity);
        panelsTelemetry.addData("RPM", currentVelocity *60 / 28);

        //graphManager.addData("Target Velocity (ticks/s)", targetVelocity);
        //graphManager.addData("Measured Velocity (ticks/s)", currentVelocity);

        panelsTelemetry.addData("Motor Power", power);
        panelsTelemetry.addLine(String.format("PID | P: %.5f, I: %.5f, D: %.5f", p, i, d));
        //graphManager.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        intake.setPower(0);
        panelsTelemetry.addLine("Shooter PID Tuner Stopped");
        panelsTelemetry.update();
    }
}