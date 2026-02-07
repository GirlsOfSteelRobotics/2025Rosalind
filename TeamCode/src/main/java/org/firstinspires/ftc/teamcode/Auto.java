package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto1", group="Robot")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap, telemetry);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();
if (opModeIsActive()) {
    chassis.moveForward(3000, telemetry);
    chassis.moveBackwards(3000, telemetry);
    chassis.moveSideways(3000, telemetry);
    chassis.turnOneWay(3000, telemetry);
        }
    }
}
