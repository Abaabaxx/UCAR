import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# 输入文件路径
IMU_DATA_FILE = 'imu_data.csv'

# 标准重力加速度 (m/s^2)
STANDARD_GRAVITY = 9.80665

# 参数手册提供的参考值
DATASHEET_PARAMS = {
    'orientation_static_accuracy_deg': 0.05  # 俯仰/横滚静态精度 (度)
}

def load_imu_data(filepath: str) -> pd.DataFrame:
    """
    加载并预处理IMU数据
    
    参数:
        filepath: CSV文件的路径
    
    返回:
        处理好的pandas DataFrame
    """
    # 读取CSV文件
    df = pd.read_csv(filepath)
    
    # 清理列名，移除可能存在的%前缀和前后空格
    df.columns = [col.strip().lstrip('%') for col in df.columns]
    
    # 将time列（纳秒时间戳）转换为相对于第一个时间戳的秒数
    first_timestamp = df['time'].iloc[0]
    df['time_sec'] = (df['time'] - first_timestamp) / 1e9
    
    return df

def analyze_angular_velocity(df: pd.DataFrame) -> dict:
    """
    分析角速度数据
    
    参数:
        df: 包含IMU数据的DataFrame
    
    返回:
        字典，包含各轴角速度的均值和方差
    """
    results = {}
    
    # 分析x、y、z三个轴的角速度
    for axis in ['x', 'y', 'z']:
        column = f'field.angular_velocity.{axis}'
        if column in df.columns:
            mean = df[column].mean()
            variance = df[column].var()
            results[axis] = {'mean': mean, 'variance': variance}
    
    return results

def analyze_linear_acceleration(df: pd.DataFrame, g: float) -> dict:
    """
    分析线性加速度数据，并考虑重力影响
    
    参数:
        df: 包含IMU数据的DataFrame
        g: 标准重力值
    
    返回:
        字典，包含各轴线性加速度的均值和方差
    """
    results = {}
    
    # 分析x、y、z三个轴的线性加速度
    for axis in ['x', 'y', 'z']:
        column = f'field.linear_acceleration.{axis}'
        if column in df.columns:
            mean = df[column].mean()
            variance = df[column].var()
            results[axis] = {'mean': mean, 'variance': variance}
    
    return results

def analyze_orientation(df: pd.DataFrame) -> dict:
    """
    分析姿态数据的稳定性
    
    参数:
        df: 包含IMU数据的DataFrame
    
    返回:
        字典，包含roll、pitch、yaw的均值和标准差（单位为度）
    """
    # 提取四元数列
    quat_columns = ['field.orientation.x', 'field.orientation.y', 'field.orientation.z', 'field.orientation.w']
    
    # 检查所有必需的列是否都存在
    if all(col in df.columns for col in quat_columns):
        # 获取四元数数据
        quats = df[quat_columns].values
        
        # 使用scipy将四元数转换为欧拉角（顺序'xyz'，单位为度）
        rot = R.from_quat(quats)
        euler_angles = rot.as_euler('xyz', degrees=True)
        
        # 计算roll, pitch, yaw的均值和标准差
        results = {
            'roll': {'mean_deg': np.mean(euler_angles[:, 0]), 'std_dev_deg': np.std(euler_angles[:, 0])},
            'pitch': {'mean_deg': np.mean(euler_angles[:, 1]), 'std_dev_deg': np.std(euler_angles[:, 1])},
            'yaw': {'mean_deg': np.mean(euler_angles[:, 2]), 'std_dev_deg': np.std(euler_angles[:, 2])}
        }
        
        return results, euler_angles
    
    # 如果缺少列，返回空结果
    return {}, np.array([])

def plot_data(df: pd.DataFrame, euler_angles: np.ndarray):
    """
    创建并显示数据的可视化图表
    
    参数:
        df: 包含IMU数据的DataFrame
        euler_angles: N行3列的numpy数组，包含roll, pitch, yaw
    """
    # 创建一个包含3个子图的Figure
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # 子图1: 绘制三轴线性加速度随时间变化的曲线
    ax1 = axes[0]
    accel_columns = ['field.linear_acceleration.x', 'field.linear_acceleration.y', 'field.linear_acceleration.z']
    if all(col in df.columns for col in accel_columns):
        ax1.plot(df['time_sec'].values, df['field.linear_acceleration.x'].values, 'r-', label='X')
        ax1.plot(df['time_sec'].values, df['field.linear_acceleration.y'].values, 'g-', label='Y')
        ax1.plot(df['time_sec'].values, df['field.linear_acceleration.z'].values, 'b-', label='Z')
        ax1.set_title('线性加速度随时间变化')
        ax1.set_xlabel('时间 (秒)')
        ax1.set_ylabel('加速度 (m/s²)')
        ax1.legend()
        ax1.grid(True)
    
    # 子图2: 绘制三轴角速度随时间变化的曲线
    ax2 = axes[1]
    gyro_columns = ['field.angular_velocity.x', 'field.angular_velocity.y', 'field.angular_velocity.z']
    if all(col in df.columns for col in gyro_columns):
        ax2.plot(df['time_sec'].values, df['field.angular_velocity.x'].values, 'r-', label='X')
        ax2.plot(df['time_sec'].values, df['field.angular_velocity.y'].values, 'g-', label='Y')
        ax2.plot(df['time_sec'].values, df['field.angular_velocity.z'].values, 'b-', label='Z')
        ax2.set_title('角速度随时间变化')
        ax2.set_xlabel('时间 (秒)')
        ax2.set_ylabel('角速度 (rad/s)')
        ax2.legend()
        ax2.grid(True)
    
    # 子图3: 绘制Roll, Pitch, Yaw欧拉角随时间变化的曲线
    ax3 = axes[2]
    if euler_angles.size > 0:
        ax3.plot(df['time_sec'].values, euler_angles[:, 0], 'r-', label='Roll')
        ax3.plot(df['time_sec'].values, euler_angles[:, 1], 'g-', label='Pitch')
        ax3.plot(df['time_sec'].values, euler_angles[:, 2], 'b-', label='Yaw')
        ax3.set_title('欧拉角随时间变化')
        ax3.set_xlabel('时间 (秒)')
        ax3.set_ylabel('角度 (度)')
        ax3.legend()
        ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

def generate_report(stats_gyro: dict, stats_accel: dict, stats_orient: dict, datasheet: dict, gravity: float):
    """
    将所有分析结果格式化并打印到控制台
    
    参数:
        stats_gyro: 角速度分析结果
        stats_accel: 线性加速度分析结果
        stats_orient: 姿态分析结果
        datasheet: 参数手册提供的参考值
        gravity: 标准重力值
    """
    print("\n" + "="*80)
    print(f"IMU 基础性能分析报告 (数据源: {IMU_DATA_FILE})")
    print("="*80)
    
    # 打印角速度分析部分
    print("\n[角速度分析]")
    print("-"*40)
    for axis, stats in stats_gyro.items():
        print(f"轴 {axis.upper()}:")
        print(f"  零偏 (Bias): {stats['mean']:.8f} rad/s")
        print(f"  方差 (Variance): {stats['variance']:.8f} (rad/s)²")
    
    # 打印线性加速度分析部分
    print("\n[线性加速度分析]")
    print("-"*40)
    for axis, stats in stats_accel.items():
        if axis == 'z':
            raw_mean = stats['mean']
            bias = raw_mean + gravity  # 考虑重力影响计算零偏
            print(f"轴 {axis.upper()}:")
            print(f"  原始均值: {raw_mean:.8f} m/s²")
            print(f"  零偏 (Bias): {bias:.8f} m/s² (已补偿重力)")
            print(f"  方差 (Variance): {stats['variance']:.8f} (m/s²)²")
        else:
            print(f"轴 {axis.upper()}:")
            print(f"  零偏 (Bias): {stats['mean']:.8f} m/s²")
            print(f"  方差 (Variance): {stats['variance']:.8f} (m/s²)²")
    
    # 打印姿态稳定性分析部分
    print("\n[姿态稳定性分析]")
    print("-"*40)
    for angle_type, stats in stats_orient.items():
        if angle_type in ['roll', 'pitch']:
            std_dev = stats['std_dev_deg']
            datasheet_value = datasheet.get('orientation_static_accuracy_deg', 'N/A')
            
            comparison = "与手册指标相符" if std_dev <= datasheet_value else "与手册指标有差异"
            
            print(f"{angle_type.capitalize()}:")
            print(f"  均值: {stats['mean_deg']:.4f}°")
            print(f"  标准差: {std_dev:.4f}° (手册指标: {datasheet_value}°) - {comparison}")
        else:
            print(f"{angle_type.capitalize()}:")
            print(f"  均值: {stats['mean_deg']:.4f}°")
            print(f"  标准差: {stats['std_dev_deg']:.4f}°")
    
    # 打印最终EKF配置建议部分
    print("\n[EKF配置建议 (用于robot_localization)]")
    print("-"*60)
    print("将以下协方差值填入您的 aunch 文件的 `imu0_config` 部分。")
    print("注意: 协方差是一个9元素的数组 [v_xx, 0, 0, 0, v_yy, 0, 0, 0, v_zz]。")

    # --- Orientation Covariance ---
    print("\n# 姿态协方差 (orientation_covariance):")
    orient_cov = [0.0] * 9
    # Roll 和 Pitch 的方差，从手册的标准差（度）计算
    roll_std_rad = np.deg2rad(stats_orient['roll']['std_dev_deg'])
    pitch_std_rad = np.deg2rad(stats_orient['pitch']['std_dev_deg'])
    orient_cov[0] = roll_std_rad**2
    orient_cov[4] = pitch_std_rad**2
    # Yaw 的方差，设置为一个非常大的值，表示不信任其绝对读数
    orient_cov[8] = 1e6  # A large number
    print(f"orientation_covariance: [{orient_cov[0]:.8e}, 0.0, 0.0, 0.0, {orient_cov[4]:.8e}, 0.0, 0.0, 0.0, {orient_cov[8]:.1e}]")

    # --- Angular Velocity Covariance ---
    print("\n# 角速度协方差 (angular_velocity_covariance):")
    ang_vel_cov = [0.0] * 9
    ang_vel_cov[0] = stats_gyro['x']['variance']
    ang_vel_cov[4] = stats_gyro['y']['variance']
    ang_vel_cov[8] = stats_gyro['z']['variance']
    print(f"angular_velocity_covariance: [{ang_vel_cov[0]:.8e}, 0.0, 0.0, 0.0, {ang_vel_cov[4]:.8e}, 0.0, 0.0, 0.0, {ang_vel_cov[8]:.8e}]")

    # --- Linear Acceleration Covariance ---
    print("\n# 线性加速度协方差 (linear_acceleration_covariance):")
    lin_accel_cov = [0.0] * 9
    lin_accel_cov[0] = stats_accel['x']['variance']
    lin_accel_cov[4] = stats_accel['y']['variance']
    lin_accel_cov[8] = stats_accel['z']['variance']
    print(f"linear_acceleration_covariance: [{lin_accel_cov[0]:.8e}, 0.0, 0.0, 0.0, {lin_accel_cov[4]:.8e}, 0.0, 0.0, 0.0, {lin_accel_cov[8]:.8e}]")

    print("\n" + "="*80)
    print("报告完成。")

def main():
    """
    主函数，按顺序调用所有函数完成分析流程
    """
    try:
        # 加载数据
        df = load_imu_data(IMU_DATA_FILE)
        
        # 获取数据时长
        data_duration = df['time_sec'].iloc[-1] - df['time_sec'].iloc[0]
        print(f"加载了 {len(df)} 个数据点，总时长: {data_duration:.2f} 秒")
        
        # 分析角速度
        stats_gyro = analyze_angular_velocity(df)
        
        # 分析线性加速度
        stats_accel = analyze_linear_acceleration(df, STANDARD_GRAVITY)
        
        # 分析姿态
        stats_orient, euler_angles = analyze_orientation(df)
        
        # 生成报告
        generate_report(stats_gyro, stats_accel, stats_orient, DATASHEET_PARAMS, STANDARD_GRAVITY)
        
        # 绘制图表
        plot_data(df, euler_angles)
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()