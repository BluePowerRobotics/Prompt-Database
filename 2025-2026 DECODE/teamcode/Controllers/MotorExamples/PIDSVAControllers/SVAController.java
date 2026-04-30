package org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers;

/**
 * SVAController类实现了SVA（静态摩擦、速度、加速度）前馈控制器
 * 用于计算电机控制的前馈项，提高控制精度
 */
public class SVAController {
    /** 静态摩擦系数 */
    double kS;
    /** 速度系数 */
    double kV;
    /** 加速度系数 */
    double kA;
    
    /**
     * 构造函数，初始化SVA参数
     * @param kS 静态摩擦系数
     * @param kV 速度系数
     * @param kA 加速度系数
     */
    public SVAController(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
    
    /**
     * 计算SVA前馈输出
     * @param velocity 当前速度
     * @param acceleration 当前加速度
     * @return SVA前馈输出
     */
    public double calculate(double velocity, double acceleration){
        // 计算静态摩擦项 + 速度项 + 加速度项
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }
    
    /**
     * 设置SVA参数
     * @param kS 静态摩擦系数
     * @param kV 速度系数
     * @param kA 加速度系数
     */
    public void setSVA(double kS, double kV, double kA){
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
}
