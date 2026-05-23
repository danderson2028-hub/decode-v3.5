package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Velocity Tuning")

public class Autoaim extends NextFTCOpMode {

    public Autoaim() {
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double flywheelPower = 0.0;
    public static double hoodAngle = 0.0;

    private MotorEx flywheel;

    private MotorEx flywheelTwo;

    private Servo hood;

    public void onInit() {
        flywheel = new MotorEx("shooter");
        flywheelTwo = new MotorEx("shooterTwo");
        hood = hardwareMap.get(Servo.class, "hood");

        panelsTelemetry.addLine("Flywheel + Hood Teleop Initialized");
        panelsTelemetry.update();
    }

    @Override
    public void onUpdate() {
        // Directly set flywheel power from dashboard value
        flywheel.setPower(Math.max(-1.0, Math.min(1.0, flywheelPower)));
        flywheelTwo.setPower(Math.max(-1.0, Math.min(1.0, flywheelPower)));

        // Set hood angle from dashboard value
        hood.setPosition(Math.max(0.0, Math.min(1.0, hoodAngle)));

        // Telemetry
        panelsTelemetry.addData("Flywheel Power", flywheelPower);
        panelsTelemetry.addData("Flywheel Velocity (ticks/s)", flywheel.getVelocity());
        panelsTelemetry.addData("Flywheel RPM", flywheel.getVelocity() * 60.0 / 28.0);
        panelsTelemetry.addData("Hood Angle", hoodAngle);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        flywheel.setPower(0);
        panelsTelemetry.addLine("Flywheel + Hood Teleop Stopped");
        panelsTelemetry.update();
    }

}
