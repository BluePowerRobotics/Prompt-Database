package org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers;

import java.util.HashMap;
import java.util.Map;

/**
 * PIDSVAController类实现了带有SVA前馈的PID控制器
 * 支持多slot配置，可根据不同场景切换参数
 */
public class PIDSVAController {
    /** 存储不同slot的配置 */
    private final Map<Integer, SlotConfig> slots = new HashMap<>();
    /** 当前使用的slot */
    private int currentSlot = 0;
    /** 积分值 */
    private double integral = 0;
    /** 上一次的误差值 */
    private double previousError = 0;

    /**
     * 快速设置默认slot(0号slot)的PID和SVA参数
     * @param config SlotConfig配置对象
     * @return 当前PIDSVAController实例，用于链式调用
     */
    public PIDSVAController withSlot0(SlotConfig config) {
        currentSlot = 0;
        return withSlot(0, config);
    }

    /**
     * 设置第n个slot的PID和SVA参数
     * @param slot slot编号
     * @param config SlotConfig配置对象
     * @return 当前PIDSVAController实例，用于链式调用
     */
    public PIDSVAController withSlot(int slot, SlotConfig config) {
        currentSlot = slot;
        slots.put(slot, config);
        return this;
    }

    /**
     * 切换到第n个slot，重置积分和微分状态
     * @param slot 要切换到的slot编号
     * @throws IllegalArgumentException 如果slot未配置
     */
    public void setSlot(int slot) {
        if (!slots.containsKey(slot)) throw new IllegalArgumentException("Slot not configured");
        currentSlot = slot;
        integral = 0;
        previousError = 0;
    }

    /**
     * 重置默认slot(0号slot)的配置
     * @param config 新的SlotConfig配置对象
     * @throws IllegalArgumentException 如果0号slot未配置
     */
    public void resetSlot(SlotConfig config) {
        if (!slots.containsKey(0)) throw new IllegalArgumentException("Slot not configured");
        slots.put(0, config);
    }
    
    /**
     * 重置指定slot的配置
     * @param slot slot编号
     * @param config 新的SlotConfig配置对象
     * @throws IllegalArgumentException 如果指定slot未配置
     */
    public void resetSlot(int slot, SlotConfig config) {
        if (!slots.containsKey(slot)) throw new IllegalArgumentException("Slot not configured");
        slots.put(slot, config);
    }


    /**
     * 快捷方法：简单速度闭环 / 简单位置闭环
     * VelCycle 为真：setpoint 作为前馈速度项，忽略加速度项
     * VelCycle 为假：前馈速度和加速度项均为0
     * @param setpoint 目标值（速度或位置）
     * @param measurement 当前值（速度或位置）
     * @param dt 时间间隔（秒）
     * @param VelCycle 是否为速度闭环
     * @return 控制器输出
     */
    public double calculate(double setpoint, double measurement, double dt, boolean VelCycle) {
        if(VelCycle){
            return calculate(setpoint, measurement, setpoint, 0.0, dt);
        }
        else{
            return calculate(setpoint, measurement, 0.0, 0.0, dt);
        }
    }

    /**
     * 完整PIDSVA闭环，计算输出
     * @param setpoint 目标值（位置或速度）
     * @param measurement 当前值（位置或速度）
     * @param velocity 当前速度（用于SVA前馈）
     * @param acceleration 当前加速度（用于SVA前馈）
     * @param dt 时间间隔（秒）
     * @return 控制器输出
     */
    public double calculate(double setpoint, double measurement, double velocity, double acceleration, double dt) {
        // 获取当前slot的配置
        SlotConfig cfg = slots.get(currentSlot);
        // 计算误差
        double error = setpoint - measurement;
        // 计算积分
        integral += error * dt;
        // 积分限幅
        if (cfg != null && integral > cfg.maxI) integral = cfg.maxI;
        if (cfg != null && integral < -cfg.maxI) integral = -cfg.maxI;
        // 计算微分
        double derivative = (error - previousError) / dt;
        // 更新上一次误差
        previousError = error;
        // 计算PID输出
        double pid = cfg.kP * error + cfg.kI * integral + cfg.kD * derivative;
        // 计算SVA前馈输出
        double sva = cfg.kS * Math.signum(velocity) + cfg.kV * velocity + cfg.kA * acceleration;
        // 计算总输出
        double output = pid + sva;
        // 输出限幅
        if (output > cfg.outputMax) output = cfg.outputMax;
        if (output < cfg.outputMin) output = cfg.outputMin;
        // 返回输出
        return output;
    }

    /**
     * 重置积分和微分状态（不切换slot）
     */
    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
