package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.graph.GraphManager;
//import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackElement;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Position Tuning")
public class PositionPidTuning extends NextFTCOpMode {
    public PositionPidTuning(){
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
    public static double s = 0.0;
    private double prevS;

    // This object links live to the dashboard.
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);


    // ---- Tunable target velocity ----
    public static double target = 0.0; // ticks per second


    // ---- Hardware ----
    private MotorEx intake;

    // ---- Control System ----
    private ControlSystem controller;

    @Override
    public void onInit() {
        panelsTelemetry.addLine("Shooter PID Tuner Initialized");
        panelsTelemetry.update();

        // Initialize motor
        intake = new MotorEx("turret");
        intake.setCurrentPosition(0);

        //intake.zero();

        // Build controller with velocity PID using live-updating coefficients
        controller = ControlSystem.builder()
                .posPid(coefficients)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        intake.setCurrentPosition(0);
        intake.zero();
        panelsTelemetry.update();
    }

    @Override
    public void onUpdate() {
        if(p!=coefficients.kP || i!=coefficients.kI || d!=coefficients.kD|| s!=prevS) {
            coefficients.kP = p;
            coefficients.kI = i;
            coefficients.kD = d;

            controller = ControlSystem.builder()
                    .posPid(coefficients)
                    .basicFF(0,0,s)
                    .build();
        }


        // Optional: only rebuild if values changed, or rebuild every loop for simplicity


        // === Handle Gamepad Input ===

        // === Control Loop ===
        double currentVelocity = intake.getVelocity(); // ticks per second
        double currentPosition = intake.getCurrentPosition();

        // Set goal (position not used here, but required by KineticState)
        controller.setGoal(new KineticState(target, Math.signum(target - currentPosition)));

        // Calculate motor power using current state
        double power = controller.calculate(intake.getState());

        // Clamp power for safety
        power = Math.max(-1.0, Math.min(1.0, power));

        intake.setPower(power);


        // === Telemetry ===
        panelsTelemetry.addData("Target", target);
        panelsTelemetry.addData("Pos", currentPosition);


        //graphManager.addData("Target Velocity (ticks/s)", targetVelocity);
        //graphManager.addData("Measured Velocity (ticks/s)", currentVelocity);
        prevS=s;
        panelsTelemetry.addData("Motor Power", power);
        panelsTelemetry.addLine(String.format("PID | P: %.5f, I: %.5f, D: %.5f", p, i, d));
        //graphManager.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        intake.setPower(0);

        panelsTelemetry.update();
    }
}