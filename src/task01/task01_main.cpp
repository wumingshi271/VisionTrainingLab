#include "task01/armor_pose_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
    struct Task01Config {
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        task01::ArmorSize smallArmor;
        task01::ArmorSize largeArmor;
    };

    struct SampleData {
        task01::ArmorType armorType = task01::ArmorType::SMALL;
        std::vector<cv::Point2f> corners;
    };

    std::string toLowercase(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        return value;
    }

    double readNumber(const cv::FileNode &node, const std::string &fieldName) {
        if (node.isNone() || (!node.isInt() && !node.isReal())) {
            throw std::runtime_error("配置项 " + fieldName + " 必须是数字");
        }

        const double value = static_cast<double>(node);
        if (!std::isfinite(value)) {
            throw std::runtime_error("配置项 " + fieldName + " 不能是 NaN 或者 Inf");
        }
        return value;
    }

    std::string readString(const cv::FileNode &node, const std::string &fieldName) {
        if (node.isNone() || !node.isString()) {
            throw std::runtime_error("配置项 " + fieldName + " 必须是字符串");
        }
        return static_cast<std::string>(node);
    }

    std::vector<double> readNumberSequence(
        const cv::FileNode &node,
        const std::string &fieldName) {
        if (node.isNone() || !node.isSeq()) {
            throw std::runtime_error("配置项 " + fieldName + " 必须是数字序列");
        }

        std::vector<double> values;
        node >> values;
        for (const double value : values) {
            if (!std::isfinite(value)) {
                throw std::runtime_error("配置项 " + fieldName + " 不能包含 NaN 或者 Inf");
            }
        }
        return values;
    }

    double readPositiveDimension(
        const cv::FileNode &armorNode,
        const std::string &fieldName,
        double unitScale) {
        const double value = readNumber(armorNode[fieldName], "armor." + fieldName);
        if (value <= 0.0) {
            throw std::runtime_error("配置项 armor." + fieldName + " 必须大于零");
        }

        const double valueInMeters = value * unitScale;
        if (!std::isfinite(valueInMeters) || valueInMeters <= 0.0) {
            throw std::runtime_error("配置项 armor." + fieldName + " 转换为 meter 后无效");
        }
        return valueInMeters;
    }

    Task01Config loadConfig(const std::string &configPath) {
        cv::FileStorage fileStorage(configPath, cv::FileStorage::READ);
        if (!fileStorage.isOpened()) {
            throw std::runtime_error("无法打开配置文件: " + configPath);
        }

        Task01Config config;
        const std::vector<double> cameraValues = readNumberSequence(
            fileStorage["camera_matrix"], "camera_matrix");
        if (cameraValues.size() != 9) {
            throw std::runtime_error("相机矩阵必须有 9 个值");
        }

        config.cameraMatrix = cv::Mat(3, 3, CV_64F);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                config.cameraMatrix.at<double>(row, column) =
                    cameraValues[static_cast<std::size_t>(row * 3 + column)];
            }
        }

        if (config.cameraMatrix.at<double>(0, 0) <= 0.0 ||
            config.cameraMatrix.at<double>(1, 1) <= 0.0) {
            throw std::runtime_error("相机焦距必须大于零");
        }

        const std::vector<double> distValues = readNumberSequence(
            fileStorage["distort_coeffs"], "distort_coeffs");
        if (!distValues.empty() && distValues.size() != 4 && distValues.size() != 5 &&
            distValues.size() != 8 && distValues.size() != 12 && distValues.size() != 14) {
            throw std::runtime_error("畸变参数必须包含 4、5、8、12 或 14 个值");
        }
        if (!distValues.empty()) {
            config.distCoeffs = cv::Mat(distValues).reshape(1, 1).clone();
        }

        const cv::FileNode armorNode = fileStorage["armor"];
        if (armorNode.isNone() || !armorNode.isMap()) {
            throw std::runtime_error("配置项 armor 必须是映射节点");
        }

        const std::string unit = toLowercase(readString(armorNode["unit"], "armor.unit"));
        double unitScale = 0.0;
        if (unit == "meter" || unit == "meters" || unit == "m") {
            unitScale = 1.0;
        } else if (unit == "centimeter" || unit == "centimeters" || unit == "cm") {
            unitScale = 0.01;
        } else if (unit == "millimeter" || unit == "millimeters" || unit == "mm") {
            unitScale = 0.001;
        } else {
            throw std::runtime_error("不支持的装甲板尺寸单位: " + unit);
        }

        config.smallArmor.width = readPositiveDimension(armorNode, "small_width", unitScale);
        config.smallArmor.height = readPositiveDimension(armorNode, "small_height", unitScale);
        config.largeArmor.width = readPositiveDimension(armorNode, "large_width", unitScale);
        config.largeArmor.height = readPositiveDimension(armorNode, "large_height", unitScale);
        return config;
    }

    SampleData loadSample(const std::string &samplePath) {
        cv::FileStorage fileStorage(samplePath, cv::FileStorage::READ);
        if (!fileStorage.isOpened()) {
            throw std::runtime_error("无法打开样例文件: " + samplePath);
        }

        const cv::FileNode sampleNode = fileStorage["sample"];
        if (sampleNode.isNone() || !sampleNode.isMap()) {
            throw std::runtime_error("配置项 sample 必须是映射节点");
        }

        const std::string armorType = toLowercase(
            readString(sampleNode["armor_type"], "sample.armor_type"));
        SampleData sample;
        if (armorType == "small") {
            sample.armorType = task01::ArmorType::SMALL;
        } else if (armorType == "large") {
            sample.armorType = task01::ArmorType::LARGE;
        } else {
            throw std::runtime_error("sample.armor_type 必须是 small 或 large");
        }

        const cv::FileNode cornersNode = sampleNode["corners"];
        if (cornersNode.isNone() || !cornersNode.isSeq() || cornersNode.size() != 4) {
            throw std::runtime_error("sample.corners 必须包含四个二维点");
        }

        sample.corners.reserve(4);
        for (int index = 0; index < 4; ++index) {
            const cv::FileNode cornerNode = cornersNode[index];
            if (cornerNode.isNone() || !cornerNode.isSeq() || cornerNode.size() != 2) {
                throw std::runtime_error("sample.corners 中的每个点必须包含两个坐标");
            }

            const double x = readNumber(cornerNode[0], "sample.corners.x");
            const double y = readNumber(cornerNode[1], "sample.corners.y");
            if (std::abs(x) > std::numeric_limits<float>::max() ||
                std::abs(y) > std::numeric_limits<float>::max()) {
                throw std::runtime_error("sample.corners 坐标超出 Point2f 范围");
            }
            sample.corners.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
        return sample;
    }

    const char *armorTypeName(task01::ArmorType armorType) {
        switch (armorType) {
            case task01::ArmorType::SMALL:
                return "small";
            case task01::ArmorType::LARGE:
                return "large";
        }
        return "unknown";
    }

    void printCameraMatrix(const cv::Mat &cameraMatrix) {
        std::cout << "[task01] camera matrix:\n";
        for (int row = 0; row < cameraMatrix.rows; ++row) {
            std::cout << "  [" << cameraMatrix.at<double>(row, 0) << ", "
                      << cameraMatrix.at<double>(row, 1) << ", "
                      << cameraMatrix.at<double>(row, 2) << "]\n";
        }
    }

    void printPoseResult(const task01::PoseResult &result) {
        std::cout << "[task01] rvec: [" << result.rvec[0] << ", " << result.rvec[1]
                  << ", " << result.rvec[2] << "] rad\n";
        std::cout << "[task01] tvec camera: x=" << result.tvec[0]
                  << "m y=" << result.tvec[1] << "m z=" << result.tvec[2] << "m\n";
        std::cout << "[task01] distance: " << result.distance << "m\n";
        std::cout << "[task01] yaw/pitch/roll: " << result.yaw_deg << " / "
                  << result.pitch_deg << " / " << result.roll_deg << " deg\n";
        std::cout << "[task01] reprojection error: "
                  << result.reprojection_error_px << " px\n";
    }

    cv::Point toCanvasPoint(const cv::Point2f &point, double offsetX, double offsetY) {
        return cv::Point(
            cvRound(static_cast<double>(point.x) + offsetX),
            cvRound(static_cast<double>(point.y) + offsetY));
    }

    void drawVisualization(
        const Task01Config &config,
        const SampleData &sample,
        const task01::PoseResult &result,
        const std::string &outputPath) {
        constexpr int kMargin = 80;
        double minX = 0.0;
        double minY = 0.0;
        double maxX = 1280.0;
        double maxY = 720.0;
        for (const cv::Point2f &point : sample.corners) {
            minX = std::min(minX, static_cast<double>(point.x));
            minY = std::min(minY, static_cast<double>(point.y));
            maxX = std::max(maxX, static_cast<double>(point.x));
            maxY = std::max(maxY, static_cast<double>(point.y));
        }

        const double offsetX = minX < 0.0 ? -minX + kMargin : 0.0;
        const double offsetY = minY < 0.0 ? -minY + kMargin : 0.0;
        const int width = std::max(kMargin * 2,
            static_cast<int>(std::ceil(maxX + offsetX + kMargin)));
        const int height = std::max(kMargin * 2,
            static_cast<int>(std::ceil(maxY + offsetY + kMargin)));
        cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(30, 30, 30));

        for (std::size_t index = 0; index < sample.corners.size(); ++index) {
            const cv::Point current = toCanvasPoint(sample.corners[index], offsetX, offsetY);
            const cv::Point next = toCanvasPoint(
                sample.corners[(index + 1) % sample.corners.size()], offsetX, offsetY);
            cv::line(canvas, current, next, cv::Scalar(0, 220, 255), 2, cv::LINE_AA);
            cv::circle(canvas, current, 6, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            cv::putText(
                canvas,
                "P" + std::to_string(index),
                current + cv::Point(8, -8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(0, 255, 255),
                2,
                cv::LINE_AA);
        }

        for (const cv::Point2f &point : result.reprojected_points) {
            cv::circle(
                canvas,
                toCanvasPoint(point, offsetX, offsetY),
                4,
                cv::Scalar(255, 0, 255),
                2,
                cv::LINE_AA);
        }

        const std::vector<cv::Point3f> axisObjectPoints{
            cv::Point3f(0.0f, 0.0f, 0.0f),
            cv::Point3f(0.10f, 0.0f, 0.0f),
            cv::Point3f(0.0f, 0.10f, 0.0f),
            cv::Point3f(0.0f, 0.0f, 0.10f)
        };
        std::vector<cv::Point2f> axisImagePoints;
        cv::projectPoints(
            axisObjectPoints,
            result.rvec,
            result.tvec,
            config.cameraMatrix,
            config.distCoeffs,
            axisImagePoints);

        const cv::Point axisOrigin = toCanvasPoint(axisImagePoints[0], offsetX, offsetY);
        const cv::Scalar axisColors[] = {
            cv::Scalar(0, 0, 255),
            cv::Scalar(0, 255, 0),
            cv::Scalar(255, 0, 0)
        };
        const char *axisNames[] = {"X", "Y", "Z"};
        for (int axis = 0; axis < 3; ++axis) {
            const cv::Point axisEnd = toCanvasPoint(
                axisImagePoints[static_cast<std::size_t>(axis + 1)], offsetX, offsetY);
            cv::line(canvas, axisOrigin, axisEnd, axisColors[axis], 3, cv::LINE_AA);
            cv::putText(
                canvas,
                axisNames[axis],
                axisEnd + cv::Point(8, 8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                axisColors[axis],
                2,
                cv::LINE_AA);
        }

        cv::putText(
            canvas,
            "yellow=input, magenta=reprojection, XYZ=pose axes",
            cv::Point(24, 36),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(240, 240, 240),
            2,
            cv::LINE_AA);
        cv::putText(
            canvas,
            "armor=" + std::string(armorTypeName(sample.armorType)),
            cv::Point(24, 68),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(240, 240, 240),
            2,
            cv::LINE_AA);

        if (!cv::imwrite(outputPath, canvas)) {
            throw std::runtime_error("无法写入可视化图片: " + outputPath);
        }
    }

    void printUsage(const char *programName) {
        std::cerr << "Usage:\n"
                  << "  " << programName
                  << " <config.yaml> <sample.yaml> [visualization.png]\n";
    }
}

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        printUsage(argv[0]);
        return 0;
    }
    if (argc != 3 && argc != 4) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string visualizationPath =
        argc == 4 ? argv[3] : "/tmp/task01_visualization.png";

    try {
        const Task01Config config = loadConfig(argv[1]);
        const SampleData sample = loadSample(argv[2]);
        const task01::ArmorPoseSolver solver(
            config.cameraMatrix,
            config.distCoeffs,
            config.smallArmor,
            config.largeArmor);

        std::cout << std::fixed << std::setprecision(6);
        std::cout << "[task01] armor type: " << armorTypeName(sample.armorType) << '\n';
        std::cout << "[task01] corner order: left_top, right_top, right_bottom, left_bottom\n";
        printCameraMatrix(config.cameraMatrix);

        const task01::PoseResult result = solver.solve(sample.corners, sample.armorType);
        if (!result.success) {
            std::cerr << "[task01] pose solve failed: " << result.error_message << '\n';
            return 1;
        }
        printPoseResult(result);

        drawVisualization(config, sample, result, visualizationPath);
        std::cout << "[task01] visualization: " << visualizationPath << '\n';

        return 0;
    } catch (const cv::Exception &exception) {
        std::cerr << "[task01] OpenCV error: " << exception.what() << '\n';
        return 1;
    } catch (const std::exception &exception) {
        std::cerr << "[task01] error: " << exception.what() << '\n';
        return 1;
    }
}
