package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.opmodes.Teleop;
import org.firstinspires.ftc.teamcode.utils.Calculations;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {
    }

    public FeedbackCRServoEx turretOne = new FeedbackCRServoEx(
            0.02,
            () -> ActiveOpMode.hardwareMap().analogInput.get("analog"),
            () -> ActiveOpMode.hardwareMap().crservo.get("turret")
    );

    public CRServoEx turretTwo = new CRServoEx("turret");

    public static boolean alignment = false;
    public static double sotmAngularOffset = 0.0;
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.03, 0.0, 0.0)
            .basicFF(0, 0, 0.05)
            .build();

    public Command runTurretToPosition(double position) {
        double clamped = MathFunctions.clamp(position, -170, 170);
        return new RunToPosition(controlSystem, clamped, 30).requires(this);
    }


    private enum AlignmentMode {
        Limelight,
        Odometry,
        OFF
    }

    public static AlignmentMode state = AlignmentMode.Odometry;

    @Override
    public void initialize() {

        resetEncoder();

    }

    double totalAngle = 0.0;
    double previousAngle = 0.0;

    public void resetEncoder() {
        previousAngle = turretOne.getCurrentPosition(); // sync to current so next delta = 0
        totalAngle = 0.0;
    }

    public void updatePosition() {
        double currentAngle = turretOne.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
    }

    public double getTurretAngleDeg() {
        return Math.toDegrees(totalAngle);
    }

    public void setCurrentPosition(double degrees) {
        totalAngle = Math.toRadians(degrees);
        previousAngle = turretOne.getCurrentPosition();
    }

    @Override
    public void periodic () {
        updatePosition();

        LLResult result = Limelight.INSTANCE.getLatestResult();
        if (!alignment) state = AlignmentMode.OFF;
        else if (result.isValid() && Math.abs(result.getFiducialResults().get(0).getTargetXDegrees()) < 10)
            state = AlignmentMode.Limelight;
        else state = AlignmentMode.Odometry;
        switch (state) {
            case Limelight:
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        double adjustedTx = fr.getTargetXDegrees(); // <-- apply offset
                        double target = MathFunctions.clamp(Math.toDegrees(totalAngle) + adjustedTx, -170, 170); // totalAngle

                        if (Math.abs(adjustedTx) > 2) {
                            controlSystem.setGoal(new KineticState(target, Math.signum(target - Math.toDegrees(totalAngle))));
                        }


                    }
                } else controlSystem.setGoal(new KineticState(Math.toDegrees(totalAngle), 0));

                break;
            case Odometry:
                Pose currentPose = Teleop.getFollower().getPose();
                double targetDeg = MathFunctions.clamp(Calculations.getTurretAngle(currentPose), -170, 170);
                controlSystem.setGoal(new KineticState(targetDeg, Math.signum(targetDeg - Math.toDegrees(totalAngle))));
                break;
            case OFF:
                break;
        }


        ActiveOpMode.telemetry().addData("Align mode", state);
        ActiveOpMode.telemetry().addData("Auto Align", alignment);
        ActiveOpMode.telemetry().addData("Turret Pos (deg)", Math.toDegrees(totalAngle));
        double power = controlSystem.calculate(new KineticState(Math.toDegrees(totalAngle), turretOne.getVelocity()));
        turretOne.setPower(power);
        turretTwo.setPower(-power);
    }
}

