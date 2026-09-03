#include "aligner_node.hpp"
#include <algorithm>
#include <cmath>
#include <tf2_eigen/tf2_eigen.hpp>

namespace {

double z_percentile(const open3d::geometry::PointCloud& pc, double percentile)
{
    if (pc.points_.empty())
        return 0.;

    std::vector<double> z_values;
    z_values.reserve(pc.points_.size());
    for (const auto& pt : pc.points_)
        z_values.push_back(pt.z());

    std::sort(z_values.begin(), z_values.end());
    const auto idx = std::min(
        z_values.size() - 1,
        static_cast<size_t>(percentile * static_cast<double>(z_values.size() - 1)));
    return z_values[idx];
}

}

void AlignerNode::auto_align(std::shared_ptr<open3d::geometry::PointCloud> sample_pc, Eigen::Isometry3d middle_trans, bool keep_last)
{
    RCLCPP_INFO(get_logger(), "Auto align...");
    // 将 radar 坐标系转换成 pnp 坐标系
    sample_pc->Transform(middle_trans.matrix());

    // 将 mesh 变成点云，传入参数为点云中点的数量

    // open3d::visualization::DrawGeometries({mesh_pc}, "mesh_pc", 1920, 1080);

    std::shared_ptr<open3d::geometry::PointCloud> to_align_pc;

    if (align_using_mesh) {
        auto mesh_pc = mesh_ori->SamplePointsUniformly(get_parameter("mesh_sample").as_int());
        to_align_pc = std::make_shared<open3d::geometry::PointCloud>();
        for (size_t i = 0; i < mesh_pc->points_.size(); ++i) {
            // TODO: 需要修改 z 轴余量大小
            // 将 mesh 中 z>0 部分截取出来
            if (mesh_pc->points_.at(i).z() >= -1e-3) {
                to_align_pc->points_.push_back(mesh_pc->points_.at(i));
                to_align_pc->normals_.push_back(mesh_pc->normals_.at(i));
            }
        }
        // open3d::visualization::DrawGeometries({cropped_mesh_pc}, "cropped_mesh_pc", 1920, 1080);
    } else {
        to_align_pc = pc_align;
    }
    to_align_pc->PaintUniformColor({1., 0.3, 0.3});
    auto min = get_parameter("crop_box.min").as_double_array();
    auto max = get_parameter("crop_box.max").as_double_array();

    open3d::geometry::AxisAlignedBoundingBox aabb = {
        { min[0], min[1], min[2] },
        { max[0], max[1], max[2] },
    };
    auto cropped = sample_pc->Crop(aabb);

    // open3d::visualization::DrawGeometries({cropped}, "cropped_sample_pc", 1920, 1080);
    // open3d::visualization::DrawGeometries({sample_pc, cropped, mesh_ori}, "cropped_origin_mesh", 1920, 1080);

    static Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
    if (!keep_last)
        trans = Eigen::Matrix4d::Identity();
    else
        cropped->Transform(trans);

    auto reg = [&](long max_iteration, double max_corr_dist, bool force_point_to_point = false) {
        if (align_using_mesh && !force_point_to_point)
            return open3d::pipelines::registration::RegistrationICP(
                *cropped, *to_align_pc,
                max_corr_dist,
                Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPlane(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, max_iteration));
        else
            return open3d::pipelines::registration::RegistrationICP(
                *cropped, *to_align_pc,
                max_corr_dist,
                Eigen::Matrix4d::Identity(),
                open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, max_iteration)); };
    Eigen::Matrix4d raw_trans = trans;
    bool aligned = false;
    const double max_corr_dist = get_parameter("max_corr_dist").as_double();
    const double coarse_corr_dist = std::max(max_corr_dist, 10.0);
    for (unsigned i = 0; i < get_parameter("max_iteration").as_int() && rclcpp::ok(); ++i) {
        auto reg_p2l = reg(1, i == 0 ? coarse_corr_dist : max_corr_dist);
        if (i == 0 && align_using_mesh && (!std::isfinite(reg_p2l.inlier_rmse_) || !std::isfinite(reg_p2l.fitness_) || reg_p2l.fitness_ <= 0.)) {
            RCLCPP_WARN(
                get_logger(),
                "Point-to-plane coarse align failed. Retry first iteration with point-to-point, max_corr_dist: %.3f",
                coarse_corr_dist);
            reg_p2l = reg(1, coarse_corr_dist, true);
        }
        if (!std::isfinite(reg_p2l.inlier_rmse_) || !std::isfinite(reg_p2l.fitness_) || reg_p2l.fitness_ <= 0.) {
            RCLCPP_WARN(
                get_logger(),
                "Stop auto align at iteration %u: invalid ICP result. RMSE: %f, Fitness: %f",
                i,
                reg_p2l.inlier_rmse_,
                reg_p2l.fitness_);
            break;
        }
        RCLCPP_INFO(get_logger(), "Align RMSE: %f, Fitness: %f", reg_p2l.inlier_rmse_, reg_p2l.fitness_);
        cropped->Transform(reg_p2l.transformation_);
        raw_trans = reg_p2l.transformation_ * raw_trans;
        aligned = true;
    }

    if (!aligned) {
        RCLCPP_ERROR(get_logger(), "Auto align failed: no valid ICP iteration.");
        return;
    }

    const double ground_z = z_percentile(*cropped, 0.01);
    const double ground_shift = -ground_z;
    cropped->Translate({ 0., 0., ground_shift });
    raw_trans(2, 3) += ground_shift;
    trans = raw_trans;
    RCLCPP_INFO(get_logger(), "Normalize aligned cloud ground z by %.6f m.", ground_shift);

    if (get_parameter("vis_auto_hold").as_bool()) {
        // 关闭上一轮仍处于保持状态的窗口，避免同一节点留下多个 Open3D 窗口。
        auto_vis_timer.reset();
        if (auto_vis) {
            auto_vis->DestroyVisualizerWindow();
            auto_vis.reset();
        }
        auto_vis = std::make_shared<open3d::visualization::Visualizer>();
        auto_vis->CreateVisualizerWindow("Auto Aligner", 1920, 1080, 50, 50);
        auto_vis->AddGeometry(cropped);
        auto_vis->AddGeometry(to_align_pc);
        auto_vis->PollEvents();
        auto_vis->UpdateRender();
    }

    // to_align_pc->PaintUniformColor({ 1, 1, 1 });
    // open3d::visualization::DrawGeometries({ cropped, cropped_mesh_pc }, "Aligned", 1920, 1080);

    // 发布 对齐过程 坐标系到最终的 world 坐标系的转换
    world_tf = std::make_shared<geometry_msgs::msg::TransformStamped>();
    world_tf->header.stamp = now();
    world_tf->header.frame_id = "middle";
    world_tf->child_frame_id = "world";
    world_tf->transform = tf2::eigenToTransform(Eigen::Affine3d(trans).inverse()).transform;

    tf_broadcaster->sendTransform(*world_tf);
    log_static_tf_yaml();
    RCLCPP_INFO(get_logger(), "aligned tf prepared");

    if (auto_vis) {
        RCLCPP_INFO(get_logger(), "Auto Aligner will stay open until its window is closed.");
        auto_vis_timer = create_wall_timer(std::chrono::milliseconds(33), [this]() {
            if (!auto_vis || !auto_vis->PollEvents()) {
                if (auto_vis)
                    auto_vis->DestroyVisualizerWindow();
                auto_vis_timer.reset();
                auto_vis.reset();
                return;
            }
            auto_vis->UpdateRender();
        });
    }
}
