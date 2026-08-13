# ifndef ARMOR_POSE_SOLVER_HPP
# define ARMOR_POSE_SOLVER_HPP

# include <opencv2/calib3d.hpp>
# include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

# include <string>
# include <vector>
#include <cmath>

namespace task01{

    /**
     * @brief 装甲板类型
     */
    enum class ArmorType{
        SMALL,    // 小装甲板
        LARGE,    // 大装甲板
    };

    /**
     * @brief 真实装甲板尺寸，单位：m
     */
    struct ArmorSize{
        double width = 0.0;    // 宽度
        double height = 0.0;   // 高度
    };

    /**
     * @brief ArmorPoseSolver 的输出结果
     */
    struct PoseResult{
        // SolvePnP 是否成功
        bool success = false;

        // 旋转向量
        cv::Vec3d rvec = cv::Vec3d(0.0, 0.0, 0.0);

        // 平移向量，单位是 m
        cv::Vec3d tvec = cv::Vec3d(0.0, 0.0, 0.0);

        // 旋转矩阵
        cv::Matx33d rotation_matrix = cv::Matx33d::eye();

        // 相机原点到装甲板原点的欧氏距离，单位是 m
        double distance = 0.0;

        // 装甲板位姿欧拉角
        // 单位 degree
        double yaw_deg = 0.0;    // 偏航角
        double pitch_deg = 0.0;  // 俯仰角
        double roll_deg = 0.0;   // 横滚角

        // PnP 求解后的重投影 RMSE
        // 单位：pixel
        double reprojection_error_px = 0.0;

        // 根据求解位姿重新投影到图像上的四个点
        std::vector<cv::Point2f> reprojected_points;

        // 失败的原因
        std::string error_message = "";
    };

    /**
     * @brief ArmorPoseSolver 类
     * 
     * 输入：
     * - 相机内参
     * - 相机畸变参数
     * - 大/小真实装甲板尺寸
     * - 图像的四个装甲板角点
     *
     * 输出：
     * - 装甲板相对于相机的位姿
     */
    class ArmorPoseSolver{
    public:
        /**
         * @brief 构造函数
         * @param camera_matrix 相机内参
         * @param dist_coeffs 相机畸变参数
         * @param small_armor 小装甲板尺寸
         * @param large_armor 大装甲板尺寸
         */
        ArmorPoseSolver(
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs,
            const ArmorSize& small_armor,
            const ArmorSize& large_armor
        );

        /**
         * @brief 求解装甲板相对于相机的位姿
         * @param image_points 图像的四个装甲板角点
         * @param armor_type 装甲板类型
         * @return 位姿结果
         */
        PoseResult solve(
            const std::vector<cv::Point2f>& image_points, 
            ArmorType armor_type
        ) const;

        /**
         * @brief 获取真实装甲板角点
         *
         * 返回顺序：
         * 
         * 0: left_top 
         * 1: right_top 
         * 2: right_bottom 
         * 3: left_bottom 
         * 
         * 坐标系： 
         * 
         * origin: 装甲板中心 
         * +X: 向右 
         * +Y: 向下 
         * +Z: 垂直于装甲板平面，向外
         *
         * @param armor_type 装甲板类型
         * @return 真实装甲板角点
         */
        std::vector<cv::Point3f> getObjectPoints(
            ArmorType armor_type
        ) const;

    private:
        /**
         * @brief 获取装甲板的物理尺寸
         * @param armor_type 装甲板类型
         * @return 装甲板尺寸
         */
        const ArmorSize& getArmorSize(
            ArmorType armor_type
        ) const;

        /**
         * @brief 验证图像的四个装甲板角点是否合法
         * @param image_points 图像的四个装甲板角点
         * @param error_message 错误信息
         * @return 是否有效
         */
        bool validateImagePoints(
            const std::vector<cv::Point2f>& image_points,
            std::string& error_message
        ) const;

        /**
         * @brief 将旋转矩阵转换为欧拉角
         * @param rotation_matrix 旋转矩阵
         * @return 欧拉角
         */
        static cv::Vec3d rotationMatrixToEulerAngles(
            const cv::Matx33d& rotation_matrix
        );

        /**
         * @brief 计算重投影误差
         * @param object_points 真实装甲板角点
         * @param image_points 图像的四个装甲板角点
         * @param rvec 旋转向量
         * @param tvec 平移向量
         * @param reprojected_points 重投影点
         * @return 重投影误差
         */
        double calculateReprojectionError(
            const std::vector<cv::Point3f>& object_points,
            const std::vector<cv::Point2f>& image_points,
            const cv::Vec3d& rvec,
            const cv::Vec3d& tvec,
            std::vector<cv::Point2f>& reprojected_points
        ) const;

    private:
        // 相机内参
        cv::Mat camera_matrix_;

        // 相机畸变参数
        cv::Mat dist_coeffs_;

        // 小装甲板尺寸
        ArmorSize small_armor_;

        // 大装甲板尺寸
        ArmorSize large_armor_;

    };

} // namespace task01

# endif // ARMOR_POSE_SOLVER_HPP
