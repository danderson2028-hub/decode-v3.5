package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Calculations.getTurretAngle;

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
        double clamped = MathFunctions.clamp(position, -270, 270);
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

    double servoPosRadians = 0.0;
    double previousAngle = 0.0;

    public void resetEncoder() {
        previousAngle = turretOne.getCurrentPosition(); // sync to current so next delta = 0
        servoPosRadians = 0.0;
    }

    public void updatePosition() {
        double currentAngle = turretOne.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        servoPosRadians += deltaAngle;

        previousAngle = currentAngle;
    }

    public double getTurretAngleDeg() {
        return (Math.toDegrees(servoPosRadians)) / 2.85;
    }

    public double getServoPosDegrees()  {
        return  (Math.toDegrees(servoPosRadians));
    }

    public void setCurrentPosition(double degrees) {
        servoPosRadians = Math.toRadians(degrees);
        previousAngle = turretOne.getCurrentPosition();
    }


     double target;
     double targetDeg;

    // 360 servo degrees is 126 turret degrees
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
                        target = MathFunctions.clamp((getTurretAngleDeg() + adjustedTx) * 2.85, -270, 270); // totalAngle

                        if (Math.abs(adjustedTx) > 2) {
                            controlSystem.setGoal(new KineticState(target, Math.signum(target - Math.toDegrees(servoPosRadians))));
                        }


                    }
                } else controlSystem.setGoal(new KineticState(Math.toDegrees(servoPosRadians), 0));

                break;
            case Odometry:
                Pose currentPose = Teleop.getFollower().getPose();
                targetDeg = getTurretAngle(currentPose);
                controlSystem.setGoal(new KineticState(targetDeg, Math.signum(targetDeg - (Math.toDegrees(servoPosRadians) * 2.85))));
                break;
            case OFF:
                break;
        }


        ActiveOpMode.telemetry().addData("Align mode", state);
        ActiveOpMode.telemetry().addData("Auto Align", alignment);
        ActiveOpMode.telemetry().addData("limelight target:", target);
        ActiveOpMode.telemetry().addData("odom target:", targetDeg);
        ActiveOpMode.telemetry().addData("Servo Pos (deg)", Math.toDegrees(servoPosRadians));
        double power = controlSystem.calculate(new KineticState(Math.toDegrees(servoPosRadians), turretOne.getVelocity()));
        turretOne.setPower(power);
        turretTwo.setPower(-power);
    }
}



