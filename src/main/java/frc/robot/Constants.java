/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {

    // ** 速度大小 ** //
    public static final double SPEED_SCALE = 0.7;  //速度乘以的倍數

    // ** 馬達 ** //
    public static final int MOTOR_LEFT_1_ID = 0;  //左馬達 1
    public static final int MOTOR_LEFT_2_ID = 1;  //左馬達 2
    public static final int MOTOR_RIGHT_1_ID = 2;  //右馬達 1
    public static final int MOTOR_RIGHT_2_ID = 3;  //左馬達 2
    //public static final int MOTOR_LEFT = 0;  //左馬達
    //public static final int MOTOR_RIGHT = 1;  //右馬達
    
    // ** 搖桿 ** //
    public static final int JOYSTICK_1_ID = 0;  //羅技
    public static final int JOYSTICK_2_ID = 1;  //Arduino
    
    // ** 羅技搖桿控制器 - 軸 ** //
    public static final int LEFT_STICK_X = 0;  //左蘑菇頭 X 軸
    public static final int LEFT_STICK_Y = 1;  //左蘑菇頭 Y 軸
    public static final int RIGHT_STICK_X = 4;  //右蘑菇頭 X 軸
    public static final int RIGHT_STICK_Y = 5;  //右蘑菇頭 Y 軸
    public static final int LEFT_TRIGGER = 2;  //LT鍵
    public static final int RIGHT_TRIGGER = 3;  //RT鍵

    // ** 羅技搖桿控制器 - 按鈕 ** //
    public static final int BUTTON_A = 1;  //按鈕 A
    public static final int BUTTON_B = 2;  //按鈕 B
    public static final int BUTTON_X = 3;  //按鈕 X
    public static final int BUTTON_Y = 4;  //按鈕 Y

    // **Arduino搖桿 - 按鈕 ** // 
    public static final int ARDUINO_BUTTON_1 = 1;
    public static final int ARDUINO_BUTTON_2 = 2;
}
