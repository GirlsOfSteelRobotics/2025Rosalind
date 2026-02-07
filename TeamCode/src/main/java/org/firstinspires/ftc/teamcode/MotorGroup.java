package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorGroup{
    private final DcMotor front;
    private final DcMotor back;

    public MotorGroup (DcMotorEx motor1, DcMotorEx motor2){
        front = motor1;
        back = motor2;
    }
    public void setPower (double power){
    front.setPower(power);
    back.setPower(power);
    }
    public void setMode (DcMotor.RunMode mode) {
        front.setMode(mode);
        back.setMode(mode);
    }

    public void setIncreasedTargetPosition(int position){
        front.setTargetPosition(front.getCurrentPosition()+position);
        back.setTargetPosition(back.getCurrentPosition()+position);
    }

    public void setDecreasedTargetPosition(int position){
        front.setTargetPosition(front.getCurrentPosition()-position);
        back.setTargetPosition(back.getCurrentPosition()-position);
    }

    public boolean isBusy (){
        return front.isBusy() || back.isBusy();
    }
}
