// #include <spdlog/spdlog.h>

#include "VoxelGrid.h"
#include "VisualizerHelper.h"

using namespace pc_detector;

inline double sqr_dis(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return (a(0) - b(0)) * (a(0) - b(0)) + (a(1) - b(1)) * (a(1) - b(1)) + (a(2) - b(2)) * (a(2) - b(2));
}

void VoxelGrid::initialize(const VoxelGridParams &params) {
    /// @brief 初始化体素网格
    voxel_size = params.voxel_size;

    bbox = open3d::geometry::AxisAlignedBoundingBox(params.grid_min, params.grid_max);
    grid_size = ((bbox.max_bound - bbox.min_bound) / voxel_size + Eigen::Vector3d::Ones()).cast<int>();
    grid = std::vector<std::vector<std::vector<bool> > >(
        grid_size[0], std::vector<std::vector<bool> >(
            grid_size[1], std::vector<bool>(
                grid_size[2], false
            )
        )
    );
    o3d_grid.voxel_size_ = voxel_size;
    o3d_grid.origin_ = bbox.min_bound - Eigen::Vector3d::Ones() * voxel_size * 0.5;
}

void VoxelGrid::occupy_by_mesh(std::shared_ptr<const Z_Map> z_map, size_t dilate_size)
{
    /// @brief 利用场地高度图占据体素网格：体素中心低于所在柱的场地高度即视为被场地遮挡
    // 原实现用 t::geometry::RaycastingScene 做"雷达→体素中心"射线遮挡判定，
    // 但 apt libopen3d-dev 0.14.1 缺该符号（0.16 才有）。场地实体都在表面以下，
    // 用 Z_Map（表面采样+KDTree 柱高度）等价近似：低于柱高度 → 占据并扩张。
    // spdlog::info("Occupying voxel grid by mesh");
    for (int i = 0; i < grid_size[0]; ++i) {
    for (int j = 0; j < grid_size[1]; ++j) {
    for (int k = 0; k < grid_size[2]; ++k) {
        Eigen::Vector3d center = get_grid_center({ i, j, k });
        if (center.z() < z_map->project_ground(center.head<2>()).z())
            dilate_occupy(Eigen::Vector3i { i, j, k }, dilate_size, { 1, 0, 0 });
    }}}
    // spdlog::info("Occupying voxel grid by mesh done");
}

void VoxelGrid::occupy_by_mesh_filter(std::shared_ptr<const open3d::geometry::TriangleMesh> mesh_f, double occupy_expand)
{
    // 原实现用 RaycastingScene::ComputeSignedDistance（0.14 缺符号），
    // 改为 legacy PointCloud::ComputePointCloudDistance（KDTree 最近表面采样点距离），
    // 体素中心到场地表面距离低于 occupy_expand 即认为贴在场地上而被占据。
    const int sample_point_num = 50000;
    // 0.14 的 SamplePointsUniformly 非 const，mesh_f 是 const 指针，这里只读采样不修改
    auto samples = std::const_pointer_cast<open3d::geometry::TriangleMesh>(mesh_f)->SamplePointsUniformly(sample_point_num);

    open3d::geometry::PointCloud centers;
    centers.points_.reserve(grid_size[0] * grid_size[1] * grid_size[2]);
    for (int i = 0; i < grid_size[0]; ++i) {
    for (int j = 0; j < grid_size[1]; ++j) {
    for (int k = 0; k < grid_size[2]; ++k) {
        centers.points_.push_back(get_grid_center({ i, j, k }));
    }}}

    auto dists = centers.ComputePointCloudDistance(*samples);
    size_t idx = 0;
    for (int i = 0; i < grid_size[0]; ++i) {
    for (int j = 0; j < grid_size[1]; ++j) {
    for (int k = 0; k < grid_size[2]; ++k) {
        if (dists[idx++] < occupy_expand) {
            occupy(Eigen::Vector3i { i, j, k }, { 0, 0, 1 });
        }
    }}}
}

void VoxelGrid::occupy_by_pc(std::shared_ptr<const open3d::geometry::PointCloud> pc, size_t dilate_size)
{
    for (const auto& pt : pc->points_) {
        dilate_occupy(pt, dilate_size, { 0, 1, 0 });
    }
}

void VoxelFilter::initialize(const VoxelGridParams& params)
{
    /// @brief 初始化体素网格
    // voxel_size = Config["PointCloud"]["voxelFilter"]["voxelSize"].as<double>();
    voxel_size = params.voxel_size;
    bbox = open3d::geometry::AxisAlignedBoundingBox(params.grid_min, params.grid_max);
    grid_size = ((bbox.GetMaxBound() - bbox.GetMinBound()) / voxel_size + Eigen::Vector3d::Ones()).cast<int>();
    grid = std::vector<std::vector<std::vector<std::pair<size_t, size_t>>>>(grid_size(0) + 1,
            std::vector<std::vector<std::pair<size_t, size_t>>>(grid_size(1) + 1,
            std::vector<std::pair<size_t, size_t>>(grid_size(2) + 1, { 0, 0 })));
}

std::vector<Eigen::Vector3d> VoxelFilter::filter(const std::vector<Eigen::Vector3d>& pcd, double rate)
{
    /// @brief 体素滤波
    if (pcd.empty())
        return {};
    // clean filter_voxel
    for (auto &i: grid) {
        for (auto &j: i) {
            j.assign(j.size(), std::make_pair((size_t) 0, (size_t) 0));
        }
    }

    std::vector<Eigen::Vector3d> filtered;

    for (auto& point : pcd)
    {
        if ((point.array() >= bbox.max_bound_.array()).any() || (point.array() < bbox.min_bound_.array()).any())
            continue; // 超出范围的点默认为占据以过滤
        Eigen::Vector3d pt_grid = (point - bbox.min_bound_) / voxel_size;
        Eigen::Vector3i idx = pt_grid.cast<int>();
        grid[idx(0)][idx(1)][idx(2)].second++;
        if (grid[idx(0)][idx(1)][idx(2)].first / double(grid[idx(0)][idx(1)][idx(2)].second) <= rate) {
            filtered.push_back(point);
            grid[idx(0)][idx(1)][idx(2)].first++;
        }
    }
    return filtered;
}
