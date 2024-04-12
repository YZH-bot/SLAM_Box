#include <numeric>   // accumulate, iota头文件
#include <execution> // 并行

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <glog/logging.h>

#include <chrono>
#include <omp.h>
#include "utils/common_type.hpp"

class ICP
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType>::Ptr CloudPtr;
    typedef pcl::PointCloud<PointType> CloudType;
    typedef Eigen::Vector3d Vec3d;

    struct Options // 注意内存对齐
    {
        bool use_initial_translation_ = false;
        int max_iteration_ = 50;
        int min_effective_pts_ = 10;
        double eps_ = 1e-5;
        double max_nn_distance_ = 1.0;
    };

private:
    Options options_;

    CloudPtr target_ = nullptr;
    CloudPtr source_ = nullptr;

    Eigen::Vector3d target_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center_ = Eigen::Vector3d::Zero();

    // doc: https://blog.csdn.net/luoyihao123456/article/details/125246830
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_ = nullptr;

    // 用于验证误差
    bool gt_set_ = false;

    void BuildTargetKdTree()
    {
        kdtree_.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_->setInputCloud(target_);
    }

public:
    ICP(){};
    ICP(Options options) : options_(options) {}

    void SetTarget(CloudPtr target)
    {
        target_ = target;
        LOG(INFO) << "target size: " << target_->points.size();
        BuildTargetKdTree();

        // 计算center
        // doc: 使用 std::accumulate
        {
            auto t_s = std::chrono::high_resolution_clock::now();
            // doc: eigen混淆操作：https://blog.csdn.net/s12k39/article/details/108393613
            // doc: lamda 函数使用引用传递参数更加高效，因为它避免了不必要的复制操作
            target_center_ = std::accumulate(target_->points.begin(), target_->points.end(), Eigen::Vector3d::Zero().eval(),
                                             [](const Vec3d &c, const PointType &pt) -> Vec3d
                                             {
                                                 return c + pt.getVector3fMap().cast<double>();
                                             }) /
                             target_->points.size();
            auto t_e = std::chrono::high_resolution_clock::now();
            auto t_c = std::chrono::duration_cast<std::chrono::milliseconds>((t_e - t_s)).count();
            LOG(INFO) << "target center: " << target_center_.transpose();
            LOG(INFO) << "std::accumulate time: " << t_c << "ms";
        }
        // doc: omp：下面的写法没有上面快
        //         {
        //             target_center_ = Vec3d::Zero();
        //             double sum_x{0.0}, sum_y{0.0}, sum_z{0.0};
        //             auto num_threads_ = omp_get_max_threads();
        //             auto t_s = std::chrono::high_resolution_clock::now();
        //             // doc: eigen混淆操作：https://blog.csdn.net/s12k39/article/details/108393613
        //             // doc: lamda 函数使用引用传递参数更加高效，因为它避免了不必要的复制操作
        // #pragma omp parallel for num_threads(num_threads_) schedule(guided, 8) reduction(+ : sum_x, sum_y, sum_z)
        //             for (size_t i = 0; i < target_->points.size(); i++)
        //             {
        //                 sum_x += target_->points[i].x;
        //                 sum_y += target_->points[i].y;
        //                 sum_z += target_->points[i].z;
        //             }
        //             target_center_.x() = sum_x;
        //             target_center_.y() = sum_y;
        //             target_center_.z() = sum_z;
        //             target_center_ = target_center_ / target_->points.size();
        //             auto t_e = std::chrono::high_resolution_clock::now();
        //             auto t_c = std::chrono::duration_cast<std::chrono::milliseconds>((t_e - t_s)).count();
        //             LOG(INFO) << "target center: " << target_center_.transpose();
        //             LOG(INFO) << "omp time: " << t_c << "ms";
        //         }
    }

    void SetSource(CloudPtr source)
    {
        source_ = source;
        LOG(INFO) << "source size: " << source_->points.size();

        source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Eigen::Vector3d::Zero().eval(),
                                         [](const Vec3d &c, const PointType &pt) -> Vec3d
                                         {
                                             return c + pt.getVector3fMap().cast<double>();
                                         }) /
                         source_->points.size();
    }

    bool AglinP2P(SE3 &init_pose)
    {
        LOG(INFO) << "AglinP2P starting";
        assert(target_ != nullptr || source_ != nullptr);

        SE3 pose = init_pose;
        if (options_.use_initial_translation_)
        {
            pose.translation() = target_center_ - source_center_;
        }

        std::vector<int> index(source_->points.size());
        iota(index.begin(), index.end(), 0);

        // doc：并发处理
        std::vector<bool> effect_pts(index.size(), false);
        std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
        std::vector<Vec3d> errors(index.size());

        // doc：迭代次数
        for (int iter = 0; iter < options_.max_iteration_; iter++)
        {
            // LOG(INFO) << "iteration = " << iter;
            // doc: 加上 std::execution::par_unseq 并行，会更快 需要链接 TBB 库
            // doc：https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag_t
            // doc：https://blog.csdn.net/Vingnir/article/details/135491314
            // ?: 为什么不把求和放到 for_each 里？ 因为必须确保线程安全
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx)
                          {
                // LOG(INFO) << "for_each id = " << idx;

                Vec3d q = source_->points[idx].getVector3fMap().cast<double>();
                Vec3d q_trans = pose * q;
                PointType p;
                p.x = q_trans.x();
                p.y = q_trans.y();
                p.z = q_trans.z();

                std::vector<int> nn;
                std::vector<float> nnDis;
                // kdtree_->radiusSearch(p, options_.max_nn_distance_, nn, nnDis);
                kdtree_->nearestKSearch(p, 1, nn, nnDis);

                if(!nn.empty()){
                    Vec3d q_nearest = target_->points[nn[0]].getVector3fMap().cast<double>();
                    effect_pts[idx] = true;
                    double dis2 = (q_nearest - q_trans).squaredNorm();
                    if (dis2 > options_.max_nn_distance_) {
                        // 点离的太远了不要
                        effect_pts[idx] = false;
                        return;
                    }

                    // doc: calculate residual
                    Vec3d e = q_nearest - q_trans;
                    Eigen::Matrix<double, 3, 6> J;
                    J.block<3, 3>(0, 0) = pose.so3().matrix() * SO3::hat(q);
                    J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
                    
                    jacobians[idx] = J;
                    errors[idx] = e;
                } });

            // 累加Hessian和error,计算dx
            // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
            double total_res = 0;
            int effective_num = 0;
            auto H_and_err = std::accumulate(
                index.begin(), index.end(), std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
                [&jacobians, &errors, &effect_pts, &total_res, &effective_num](const std::pair<Mat6d, Vec6d> &pre,
                                                                               int idx) -> std::pair<Mat6d, Vec6d>
                {
                    if (!effect_pts[idx])
                    {
                        return pre;
                    }
                    else
                    {
                        total_res += errors[idx].dot(errors[idx]);
                        effective_num++;
                        return std::pair<Mat6d, Vec6d>(pre.first + jacobians[idx].transpose() * jacobians[idx],
                                                       pre.second - jacobians[idx].transpose() * errors[idx]);
                    }
                });

            if (effective_num < options_.min_effective_pts_)
            {
                LOG(WARNING) << "effective num too small: " << effective_num;
                return false;
            }

            Mat6d H = H_and_err.first;
            Vec6d err = H_and_err.second;

            Vec6d dx = H.inverse() * err;
            pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
            pose.translation() += dx.tail<3>();

            // 更新
            LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
                      << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm();

            // if (gt_set_)
            // {
            //     double pose_error = (gt_pose_.inverse() * pose).log().norm();
            //     LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
            // }

            if (dx.norm() < options_.eps_)
            {
                LOG(INFO) << "converged, dx = " << dx.transpose();
                break;
            }
        }

        init_pose = pose;
        return true;
    }
};