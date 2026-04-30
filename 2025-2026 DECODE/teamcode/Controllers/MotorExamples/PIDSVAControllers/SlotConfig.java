package org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers;

/**
 * SlotConfig类用于配置PID和SVA控制器的参数
 * 采用建造者模式，支持链式调用设置参数
 */
public class SlotConfig {
    /** 比例系数 */
    public double kP = 0;
    /** 积分系数 */
    public double kI = 0;
    /** 微分系数 */
    public double kD = 0;
    /** 积分上限 */
    public double maxI = 1;
    /** 静态摩擦系数 */
    public double kS = 0;
    /** 速度系数 */
    public double kV = 0;
    /** 加速度系数 */
    public double kA = 0;
    /** 输出最小值 */
    public double outputMin = -1;
    /** 输出最大值 */
    public double outputMax = 1;

    /**
     * 设置比例系数
     * @param kP 比例系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKP(double kP) { this.kP = kP; return this; }
    
    /**
     * 设置积分系数
     * @param kI 积分系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKI(double kI) { this.kI = kI; return this; }
    
    /**
     * 设置微分系数
     * @param kD 微分系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKD(double kD) { this.kD = kD; return this; }
    
    /**
     * 设置积分上限
     * @param maxI 积分上限
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withMaxI(double maxI) { this.maxI = maxI; return this; }
    
    /**
     * 设置静态摩擦系数
     * @param kS 静态摩擦系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKS(double kS) { this.kS = kS; return this; }
    
    /**
     * 设置速度系数
     * @param kV 速度系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKV(double kV) { this.kV = kV; return this; }
    
    /**
     * 设置加速度系数
     * @param kA 加速度系数
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withKA(double kA) { this.kA = kA; return this; }
    
    /**
     * 设置输出范围
     * @param min 输出最小值
     * @param max 输出最大值
     * @return 当前SlotConfig实例，用于链式调用
     */
    public SlotConfig withOutputLimits(double min, double max) { this.outputMin = min; this.outputMax = max; return this; }
}

