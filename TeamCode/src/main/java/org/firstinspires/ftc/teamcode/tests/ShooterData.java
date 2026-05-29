package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterData extends NextFTCOpMode {
    public ShooterData(){
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double target = 0.0;

    public double prevTarget = 0.0;

    private MotorEx shooter;
    private MotorEx shooterTwo;
    private ControlSystem controller;

    @Override
    public void onInit() {
        panelsTelemetry.addLine("Shooter PID Tuner Initialized");
        panelsTelemetry.update();

        shooter = new MotorEx("shooter");
        shooterTwo = new MotorEx("shooterTwo");

        controller = ControlSystem.builder()
                .velPid(0, 0, 0)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        target = 0.0;
        panelsTelemetry.addLine("shooter data collection started");
        panelsTelemetry.update();

    }

    @Override
    public void onUpdate() {
        if (prevTarget != target) {
            prevTarget = target;
            controller.setGoal(new KineticState(target, Math.signum(target - shooter.getCurrentPosition())));
        }

        double output = controller.calculate(new KineticState(shooter.getCurrentPosition(), shooter.getVelocity()));
        shooter.setPower(output);
        shooterTwo.setPower(output);
        }
    }

}
