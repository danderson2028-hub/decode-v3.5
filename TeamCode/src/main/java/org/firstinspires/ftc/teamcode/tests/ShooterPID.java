package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterPID extends NextFTCOpMode {
    public ShooterPID(){
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);

    private MotorEx shooter;
    private MotorEx shooterTwo;


    //private MotorEx intake;
    public static boolean runTransfer = false;

    private ControlSystem controller;

    @Override
    public void onInit() {
        panelsTelemetry.addLine("Shooter PID Tuner Initialized");
        panelsTelemetry.update();


        controller = ControlSystem.builder()
                .velPid(0, 0, 0)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        panelsTelemetry.addLine("Shooter PID Tuner Started");
        panelsTelemetry.update();
    }

    @Override
    public void onUpdate() {

        if (p != coefficients.kP || i != coefficients.kI || d != coefficients.kD) {
            coefficients.kP = p;
            coefficients.kI = i;
            coefficients.kD = d;

            controller = ControlSystem.builder()
                    .velPid(coefficients)
                    .basicFF(f, 0, 0)
                    .build();
        }

    }
    }
