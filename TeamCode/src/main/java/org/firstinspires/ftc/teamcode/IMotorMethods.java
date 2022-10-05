package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public interface IMotorMethods {
    double getPower();
    DcMotor.Direction getDirection();
    int getTargetPosition();
    int[] getCurrentPosition();
    DcMotorController[] getController();
}