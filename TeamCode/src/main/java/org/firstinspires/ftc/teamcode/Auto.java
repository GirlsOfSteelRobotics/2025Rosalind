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
            chassis.moveForward(1000, telemetry);
            sleep(100);
            chassis.moveBackwards(1000, telemetry);
            sleep(100);
            chassis.turnRight(1000, telemetry);
            sleep(100);
            chassis.turnLeft(1000, telemetry);
            sleep(100);
            chassis.sidewaysRight(1000, telemetry);
            sleep(100);
            chassis.sidewaysLeft(1000, telemetry);
        }
    }
}
