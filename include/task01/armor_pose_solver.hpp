# ifndef ARMOR_POSE_SOLVER_HPP
# define ARMOR_POSE_SOLVER_HPP

# include <opencv2/opencv.hpp>
# include <opencv2/core/core.hpp>
# include <opencv2/highgui/highgui.hpp>
# include <opencv2/imgproc/imgproc.hpp>

class ArmorPoseSolver: public cv::Algorithm
{
public:
    ArmorPoseSolver();
    ~ArmorPoseSolver();

    virtual void solve(const cv::Mat &image, cv::Mat &pose) = 0;

    virtual void load(const std::string &filename) = 0;
    virtual void save(const std::string &filename) = 0;

    virtual void setParams(const cv::Param &params) = 0;
    virtual void getParams(cv::Param &params) = 0;

    virtual void setClassifier(const cv::Classifier &classifier) = 0;
};

class ArmorPoseSolverImpl: public ArmorPoseSolver
{
public:
    ArmorPoseSolverImpl();
    ~ArmorPoseSolverImpl();

    void solve(const cv::Mat &image, cv::Mat &pose) override;
};

ArmorPoseSolverImpl::ArmorPoseSolverImpl()
{
}

ArmorPoseSolverImpl::~ArmorPoseSolverImpl()
{
}

void ArmorPoseSolverImpl::solve(const cv::Mat &image, cv::Mat &pose)
# endif // ARMOR_POSE_SOLVER_HPP
