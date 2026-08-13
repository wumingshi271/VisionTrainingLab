//
// Created by wumingshi on 2026/7/18.
//
# include "task01/armor_pose_solver.hpp"

task01::ArmorPoseSolver::ArmorPoseSolver(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    const ArmorSize &small_armor, const ArmorSize &large_armor) {
    // 判断相机内参是否合法
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3 ||
        camera_matrix.channels() != 1 ||
        (camera_matrix.depth() != CV_32F && camera_matrix.depth() != CV_64F)) {
        throw std::invalid_argument("相机内参必须是 3×3 的单通道浮点矩阵");
    }

    // 判断装甲板尺寸是否合法
    if (!std::isfinite(small_armor.width) || !std::isfinite(small_armor.height) ||
        small_armor.width <= 0.0 || small_armor.height <= 0.0 ||
        !std::isfinite(large_armor.width) || !std::isfinite(large_armor.height) ||
        large_armor.width <= 0.0 || large_armor.height <= 0.0) {
        throw std::invalid_argument("装甲板尺寸必须为有限的正数");
    }

    // 判断畸变参数是否合法
    if (!dist_coeffs.empty() &&
        (dist_coeffs.channels() != 1 ||
         (dist_coeffs.depth() != CV_32F && dist_coeffs.depth() != CV_64F) ||
         (dist_coeffs.rows != 1 && dist_coeffs.cols != 1))) {
        throw std::invalid_argument("畸变参数格式无效");
    }

    if (!dist_coeffs.empty()) {
        const std::size_t coefficient_count = dist_coeffs.total();
        if (coefficient_count != 4 && coefficient_count != 5 && coefficient_count != 8 &&
            coefficient_count != 12 && coefficient_count != 14) {
            throw std::invalid_argument("畸变参数格式无效");
        }
    }

    cv::Mat camera_matrix_double;
    cv::Mat dist_coeffs_double;

    try {
        camera_matrix.convertTo(camera_matrix_double, CV_64F);
        if (!dist_coeffs.empty()) {
            dist_coeffs.convertTo(dist_coeffs_double, CV_64F);
        }
    } catch (const cv::Exception &e) {
        throw std::invalid_argument(std::string("相机参数格式无效：") + e.what());
    }

    const auto matrixAllFinite = [](const cv::Mat &matrix) {
        for (int row = 0; row < matrix.rows; ++row) {
            for (int column = 0; column < matrix.cols; ++column) {
                if (!std::isfinite(matrix.at<double>(row, column))) {
                    return false;
                }
            }
        }

        return true;
    };

    if (!matrixAllFinite(camera_matrix_double)) {
        throw std::invalid_argument("相机内参中存在 NaN 或者 Inf");
    }

    if (!dist_coeffs.empty() && !matrixAllFinite(dist_coeffs_double)) {
        throw std::invalid_argument("畸变参数中存在 NaN 或者 Inf");
    }

    const double focal_length_x = camera_matrix_double.at<double>(0, 0);
    const double focal_length_y = camera_matrix_double.at<double>(1, 1);
    if (focal_length_x <= 0.0 || focal_length_y <= 0.0) {
        throw std::invalid_argument("相机焦距必须为正数");
    }

    camera_matrix_ = camera_matrix.clone();
    dist_coeffs_ = dist_coeffs.clone();
    small_armor_ = small_armor;
    large_armor_ = large_armor;

}

task01::PoseResult task01::ArmorPoseSolver::solve(const std::vector<cv::Point2f> &image_points,
    ArmorType armor_type) const {
    PoseResult result;

    std::string validation_error;

    try {
        // 验证四个角点的合法性
        if (!validateImagePoints(image_points,validation_error)) {
            result.success = false;
            result.error_message = validation_error;
            return result;
        }

        const auto object_points = getObjectPoints(armor_type);

        cv::Vec3d rvec{
            0.0,
            0.0,
            0.0
        };

        cv::Vec3d tvec{
            0.0,
            0.0,
            0.0
        };

        // solvePnP 主入口
        const bool solved = cv::solvePnP(
            object_points,
            image_points,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_ITERATIVE
        );

        if (!solved) {
            result.error_message = "cv::solvePnP 失败了";
            return result;
        }

        // 判断三维向量里面所有的数是否都是有限的
        const auto allFinite = [](const cv::Vec3d &v) {
            return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
        };

        const auto matrixAllFinite = [](const cv::Matx33d &matrix) {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(matrix(row, column))) {
                        return false;
                    }
                }
            }

            return true;
        };

        const auto pointsAllFinite = [](const std::vector<cv::Point2f> &points) {
            for (const auto &point : points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                    return false;
                }
            }

            return true;
        };

        // 验证 rvec 和 tvec 的合法性
        if (!allFinite(rvec) || !allFinite(tvec)) {
            result.error_message = "solvePnP 产生了 NaN 或者 Inf";
            return result;
        }

        if (tvec[2] <= 0.0) { // 这里可以根据业务逻辑适当增加不等号右边的数
            result.error_message = "解算出来的装甲板在相机后方";
            return result;
        }

        result.rvec = rvec;
        result.tvec = tvec;

        result.distance = cv::norm(tvec);
        if (!std::isfinite(result.distance) || result.distance < 0.0) {
            result.error_message = "距离结果是 NaN 或者 Inf";
            return result;
        }

        cv::Matx33d rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        result.rotation_matrix = rotation_matrix;
        if (!matrixAllFinite(rotation_matrix)) {
            result.error_message = "旋转矩阵产生了 NaN 或者 Inf";
            return result;
        }

        const cv::Vec3d euler_angles = rotationMatrixToEulerAngles(rotation_matrix);
        if (!allFinite(euler_angles)) {
            result.error_message = "欧拉角产生了 NaN 或者 Inf";
            return result;
        }
        result.yaw_deg = euler_angles[0];
        result.pitch_deg = euler_angles[1];
        result.roll_deg = euler_angles[2];

        result.reprojection_error_px = calculateReprojectionError(
            object_points,
            image_points,
            rvec,
            tvec,
            result.reprojected_points);

        if (result.reprojected_points.size() != image_points.size()) {
            result.error_message = "重投影点数量不正确";
            return result;
        }

        if (!pointsAllFinite(result.reprojected_points)) {
            result.error_message = "重投影出来的结果是 NaN 或者 Inf";
            return result;
        }

        if (!std::isfinite(result.reprojection_error_px) || result.reprojection_error_px < 0.0) {
            result.error_message = "重投影误差是 NaN 或者 Inf";
            return result;
        }

        result.success = true;
    } catch (const cv::Exception &e) {
        result.error_message = std::string("OpenCV 位姿解算异常：") + e.what();
    } catch (const std::exception &e) {
        result.error_message = std::string("位姿解算异常：") + e.what();
    }

    return result;
}

std::vector<cv::Point3f> task01::ArmorPoseSolver::getObjectPoints(ArmorType armor_type) const {
    // 获取装甲板的尺寸
    const ArmorSize& size = getArmorSize(armor_type);

    // 获取装甲板的一半长宽
    const auto half_width = static_cast<float>(size.width / 2.0f);
    const auto half_height = static_cast<float>(size.height / 2.0f);

    return {
        {
            -half_width,
            -half_height,
            0.0f,
        },// left_up

        {
            half_width,
            -half_height,
            0.0f,
        },// right_up

        {
            half_width,
            half_height,
            0.0f,
        },// right_bottom

        {
            -half_width,
            half_height,
            0.0f,
        } // left_bottom
    };
}

const task01::ArmorSize & task01::ArmorPoseSolver::getArmorSize(ArmorType armor_type) const {
    switch (armor_type) {
        case ArmorType::SMALL:
            return small_armor_;
        case ArmorType::LARGE:
            return large_armor_;
    }

    // 理论上只有大小两种装甲板类型
    throw std::runtime_error("未知的装甲板类型");
}

bool task01::ArmorPoseSolver::validateImagePoints(const std::vector<cv::Point2f> &image_points,
    std::string &error_message) const {
    // 1. 必须正好有四个点
    if (image_points.size() != 4) {
        error_message = "必须要有四个角点才可以解算装甲板";
        return false;
    }

    // 2. 每一个坐标都必须是正常的有限的数字
    for (const auto & image_point: image_points) {
        if (!std::isfinite(image_point.x) || !std::isfinite(image_point.y)) {
            error_message = "角点坐标中存在 NaN 或者 Inf";
            return false;
        }
    }

    // 3. 四个点不能退化成一条直线
    const double area = std::abs(cv::contourArea(image_points));
    constexpr double kMinAreaPx2 = 1.0;

    if (!std::isfinite(area)) {
        error_message = "装甲板的面积无效";
        return false;
    }

    if (area < kMinAreaPx2) {
        error_message = "装甲板的面积过小";
        return false;
    }

    for (std::size_t i = 0; i < image_points.size(); ++i) {
        for (std::size_t j = i + 1; j < image_points.size(); ++j) {
            const double dx = static_cast<double>(image_points[i].x) -
                static_cast<double>(image_points[j].x);
            const double dy = static_cast<double>(image_points[i].y) -
                static_cast<double>(image_points[j].y);

            if (dx * dx + dy * dy <= 1e-12) {
                error_message = "装甲板角点不能重复";
                return false;
            }
        }
    }

    double orientation = 0.0;
    for (std::size_t i = 0; i < image_points.size(); ++i) {
        const std::size_t next = (i + 1) % image_points.size();
        const std::size_t next_next = (i + 2) % image_points.size();

        const double first_x = static_cast<double>(image_points[next].x) -
            static_cast<double>(image_points[i].x);
        const double first_y = static_cast<double>(image_points[next].y) -
            static_cast<double>(image_points[i].y);
        const double second_x = static_cast<double>(image_points[next_next].x) -
            static_cast<double>(image_points[next].x);
        const double second_y = static_cast<double>(image_points[next_next].y) -
            static_cast<double>(image_points[next].y);
        const double cross_product = first_x * second_y - first_y * second_x;

        if (std::abs(cross_product) <= 1e-12) {
            error_message = "装甲板角点不能近似共线";
            return false;
        }

        if (orientation == 0.0) {
            orientation = cross_product;
        } else if (orientation * cross_product < 0.0) {
            error_message = "装甲板角点顺序或形状无效";
            return false;
        }
    }

    return true;
}

cv::Vec3d task01::ArmorPoseSolver::rotationMatrixToEulerAngles(const cv::Matx33d &rotation_matrix) {
    const double sy = std::sqrt(
        rotation_matrix(0, 0) * rotation_matrix(0, 0) +
        rotation_matrix(1, 0) * rotation_matrix(1, 0));

    const bool singular = sy < 1e-6;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    if (!singular) {
        roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
        pitch = std::atan2(-rotation_matrix(2, 0), sy);
        yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    }
    else {
        roll = std::atan2(-rotation_matrix(1, 2), rotation_matrix(1, 1));
        pitch = std::atan2(-rotation_matrix(2, 0), sy);
        yaw = 0.0;
    }

    constexpr double rad2deg = 180.0 / CV_PI;

    return {
        yaw * rad2deg,
        pitch * rad2deg,
        roll * rad2deg
    };
}

double task01::ArmorPoseSolver::calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
    const std::vector<cv::Point2f> &image_points, const cv::Vec3d &rvec, const cv::Vec3d &tvec,
    std::vector<cv::Point2f> &reprojected_points) const {
    if (object_points.size() != image_points.size() || image_points.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 根据求出的位姿，把三维模型投影到图像
    cv::projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix_,
        dist_coeffs_,
        reprojected_points);

    if (reprojected_points.size() != image_points.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double squared_error_sum = 0.0;

    for (std::size_t i = 0; i < image_points.size(); ++i) {
        const double dx = static_cast<double>(image_points[i].x) -
            static_cast<double>(reprojected_points[i].x);

        const double dy = static_cast<double>(image_points[i].y) -
            static_cast<double>(reprojected_points[i].y);

        squared_error_sum += dx * dx + dy * dy;
    }

    const double mean_error = squared_error_sum / static_cast<double>(image_points.size());

    return std::sqrt(mean_error);
}
