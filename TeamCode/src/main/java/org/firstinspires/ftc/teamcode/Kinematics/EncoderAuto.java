package org.firstinspires.ftc.teamcode.Kinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class EncoderAuto {

    public DcMotor fl, fr, bl, br;
    public EncoderAuto(LinearOpMode opMode) {
        fl = opMode.hardwareMap.get(DcMotor.class, "DrivetrainFLeft_OdometryLeft");
        fr = opMode.hardwareMap.get(DcMotor.class, "DrivetrainFRight_OdometryRight");
        br = opMode.hardwareMap.get(DcMotor.class, "DrivetrainBRight_OdometryFront");
        bl = opMode.hardwareMap.get(DcMotor.class, "DrivetrainBLeft");

        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    // reset all encoders

    void encoder_reset_all(){
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // reset movement encoders

    void encoder_reset_wheels(){
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // movement motors runmode command

    void movement_motors_runmode(){
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // basic omnidirectional movement commands (classic & target position using ticks)

    void forward(double x){
        fl.setPower(x);
        fr.setPower(x);
        bl.setPower(x);
        br.setPower(x);
    }
    void forward_target_position(int a){
        fl.setTargetPosition(a);
        fr.setTargetPosition(a);
        br.setTargetPosition(a);
        bl.setTargetPosition(a);
    }
    void stop(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    void backward(double x){
        fl.setPower(-x);
        fr.setPower(-x);
        bl.setPower(-x);
        br.setPower(-x);
    }
    void backward_target_position(int a){
        fl.setTargetPosition(-a);
        fr.setTargetPosition(-a);
        br.setTargetPosition(-a);
        bl.setTargetPosition(-a);
    }
    void left(double x){
        fl.setPower(x);
        fr.setPower(-x);
        bl.setPower(-x);
        br.setPower(x);
    }
    void left_target_position(int a){
        fl.setTargetPosition(a);
        fr.setTargetPosition(-a);
        bl.setTargetPosition(-a);
        br.setTargetPosition(a);
    }
    void right(double x){
        fl.setPower(-x);
        fr.setPower(x);
        bl.setPower(x);
        br.setPower(-x);
    }
    void right_target_position(int a){
        fl.setTargetPosition(-a);
        fr.setTargetPosition(a);
        bl.setTargetPosition(a);
        br.setTargetPosition(-a);
    }
    void fright(double x){
        fl.setPower(0);
        fr.setPower(x);
        bl.setPower(x);
        br.setPower(0);
    }
    void fright_target_position(int a){
        fl.setTargetPosition(0);
        fr.setTargetPosition(a);
        bl.setTargetPosition(a);
        br.setTargetPosition(0);
    }
    void fleft(double x){
        fl.setPower(x);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(x);
    }
    void fleft_target_position(int a){
        fl.setTargetPosition(a);
        fr.setTargetPosition(0);
        bl.setTargetPosition(0);
        br.setTargetPosition(a);
    }
    void bright(double x){
        fl.setPower(-x);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(-x);
    }
    void bright_target_position(int a){
        fl.setTargetPosition(-a);
        fr.setTargetPosition(0);
        bl.setTargetPosition(0);
        br.setTargetPosition(-a);
    }
    void bleft(double x){
        fl.setPower(0);
        fr.setPower(-x);
        bl.setPower(-x);
        br.setPower(0);
    }
    void bleft_target_position(int a){
        fl.setTargetPosition(0);
        fr.setTargetPosition(-a);
        bl.setTargetPosition(-a);
        br.setTargetPosition(0);
    }
    void tright(double x){
        fl.setPower(x);
        fr.setPower(-x);
        bl.setPower(-x);
        br.setPower(x);
    }
    void tright_target_position(int a){
        fl.setTargetPosition(a);
        fr.setTargetPosition(-a);
        bl.setTargetPosition(-a);
        br.setTargetPosition(a);
    }
    void tleft(double x){
        fl.setPower(-x);
        fr.setPower(x);
        bl.setPower(-x);
        br.setPower(x);
    }
    void tleft_target_position(int a){
        fl.setTargetPosition(-a);
        fr.setTargetPosition(a);
        bl.setTargetPosition(-a);
        br.setTargetPosition(a);
    }

    void go_forward(int tick,double speed){
        this.encoder_reset_wheels();
        this.forward_target_position(tick);
        this.movement_motors_runmode();
        this.forward(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_backward(int tick,double speed){
        this.encoder_reset_wheels();
        this.backward_target_position(tick);
        this.movement_motors_runmode();
        this.backward(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_left(int tick,double speed){
        this.encoder_reset_wheels();
        this.left_target_position(tick);
        this.movement_motors_runmode();
        this.left(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_right(int tick,double speed){
        this.encoder_reset_wheels();
        this.right_target_position(tick);
        this.movement_motors_runmode();
        this.right(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_fright(int tick,double speed){
        this.encoder_reset_wheels();
        this.fright_target_position(tick);
        this.movement_motors_runmode();
        this.fright(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_fleft(int tick,double speed){
        this.encoder_reset_wheels();
        this.fleft_target_position(tick);
        this.movement_motors_runmode();
        this.fleft(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_bright(int tick,double speed){
        this.encoder_reset_wheels();
        this.bright_target_position(tick);
        this.movement_motors_runmode();
        this.bright(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_bleft(int tick,double speed){
        this.encoder_reset_wheels();
        this.bleft_target_position(tick);
        this.movement_motors_runmode();
        this.bleft(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_tleft(int tick,double speed){
        this.encoder_reset_wheels();
        this.tleft_target_position(tick);
        this.movement_motors_runmode();
        this.tleft(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
    void go_tright(int tick,double speed){
        this.encoder_reset_wheels();
        this.tright_target_position(tick);
        this.movement_motors_runmode();
        this.tright(speed);
        while(this.fl.isBusy()||this.bl.isBusy()||this.fr.isBusy()||this.br.isBusy()){}
    }
}