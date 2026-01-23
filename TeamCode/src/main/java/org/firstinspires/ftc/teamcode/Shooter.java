package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
private final DcMotor shooterWheel;
private final DcMotor orangeWheels;
private final DcMotor intakeMain;
private final DcMotor secondIntake;

public Shooter (HardwareMap hm){
    shooterWheel = hm.get (DcMotor.class, "ShooterWheel");
    orangeWheels = hm.get(DcMotor.class, "feederWheel");
    intakeMain = hm.get(DcMotor.class, "IntakeFast");
    secondIntake = hm.get(DcMotor.class, "ballPusher");
}
}
