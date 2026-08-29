#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace pc_detector {

struct BoundingBox {
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;

    BoundingBox() = default;
    // 用于 Open3D 向 PCL 过渡
    BoundingBox(const open3d::geometry::AxisAlignedBoundingBox& aabb)
    {
        min_bound = aabb.GetMinBound();
        max_bound = aabb.GetMaxBound();
    }
    open3d::geometry::AxisAlignedBoundingBox to_aabb() const { return { min_bound, max_bound }; }
};

struct Z_Map {
    using SharedPtr = std::shared_ptr<Z_Map>;

    open3d::geometry::AxisAlignedBoundingBox bbox;
    double voxel_size;
    Eigen::Vector3i grid_size;
    Eigen::MatrixXf z_map;

    Z_Map(Eigen::Vector3d grid_min, Eigen::Vector3d grid_max, double voxel_size, std::shared_ptr<const open3d::geometry::TriangleMesh> mesh)
        : bbox(grid_min, grid_max)
        , voxel_size(voxel_size)
        , grid_size(((grid_max - grid_min) / voxel_size + Eigen::Vector3d::Ones()).cast<int>())
        , z_map(Eigen::MatrixXf::Zero(grid_size[0], grid_size[1]))
    {
        if (!mesh || mesh->IsEmpty()) {
            return;
        }
        // 表面均匀采样，按 XY 栅格取最低表面点作为该柱的地面高度。
        // 原实现从 z=max 的查询点找 3D 最近邻，容易被 STL 中的高处结构吸走，
        // 导致整根柱子低于该高度的点都被误判为静态占用。
        const int sample_point_num = 100000;
        auto samples = std::const_pointer_cast<open3d::geometry::TriangleMesh>(mesh)->SamplePointsUniformly(sample_point_num);
        Eigen::MatrixXi hit_count = Eigen::MatrixXi::Zero(grid_size[0], grid_size[1]);
        for (const auto& pt : samples->points_) {
            if ((pt.array() >= bbox.max_bound_.array()).any() || (pt.array() < bbox.min_bound_.array()).any())
                continue;
            Eigen::Vector2d pt_grid = (pt.head<2>() - bbox.min_bound_(Eigen::seq(0, 1))) / voxel_size;
            Eigen::Vector2i idx = pt_grid.cast<int>();
            if ((idx.array() >= grid_size(Eigen::seq(0, 1)).array()).any() || (idx.array() < 0).any())
                continue;
            if (hit_count(idx(0), idx(1)) == 0 || pt.z() < z_map(idx(0), idx(1)))
                z_map(idx(0), idx(1)) = static_cast<float>(pt.z());
            hit_count(idx(0), idx(1))++;
        }
        // 未采样到的格子保持 0m 作为保守地面高度，避免被高处模型误填。
        for (int i = 0; i < grid_size[0]; ++i) {
            for (int j = 0; j < grid_size[1]; ++j) {
                if (hit_count(i, j) == 0)
                    z_map(i, j) = 0.0f;
            }
        }
    }
    Eigen::Vector3d project_ground(const Eigen::Vector2d& pt) const
    {
        /// @brief 将点投影到地面
        Eigen::Vector2d pt_grid = (pt - bbox.min_bound_(Eigen::seq(0, 1))) / voxel_size;
        Eigen::Vector2i pt_grid_int = pt_grid.cast<int>();
        if ((pt_grid_int.array() >= grid_size(Eigen::seq(0, 1)).array()).any() || (pt_grid_int.array() < 0).any())
            return Eigen::Vector3d(pt(0), pt(1), 0);
        else
            return Eigen::Vector3d(pt(0), pt(1), z_map(pt_grid_int(0), pt_grid_int(1)));
    }
    Eigen::Vector3d get_grid_center(const Eigen::Vector3i& grid_idx) const
    {
        /// @brief 获取体素中心点坐标
        return bbox.min_bound_ + (grid_idx.cast<double>() + Eigen::Vector3d::Ones() * 0.5) * voxel_size;
    }
};
}
