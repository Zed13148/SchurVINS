// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#ifndef SVO_DIRECT_FEATURE_ALIGNMENT_H_
#define SVO_DIRECT_FEATURE_ALIGNMENT_H_

#include <Eigen/Core>                     // 用于矩阵和向量操作的Eigen库
#include <opencv2/core/core.hpp>          // OpenCV库的核心功能
#include <svo/common/types.h>             // SVO项目中的一些通用类型定义

namespace svo {

/// Subpixel refinement of a reference feature patch with the current image.
/// Implements the inverse-compositional approach (see "Lucas-Kanade 20 Years on"
/// paper by Baker.
// 其中包含了一些用于特征对齐的函数。注释中提到，这些函数实现了逆组成方法（inverse-compositional approach）
namespace feature_alignment {

// 用于在当前图像中对齐一个参考特征块。它允许在指定方向上移动特征块，并且可以选择性地估计仿射参数（offset和gain）
bool align1D(
    const cv::Mat& cur_img,                                 // 当前图像
    const Eigen::Ref<GradientVector>& dir,                  // direction in which the patch is allowed to move 允许特征块移动的方向
    uint8_t* ref_patch_with_border,                         // 带边界的参考特征块
    uint8_t* ref_patch,                                     // 参考特征块
    const int n_iter,                                       // 迭代次数
    const bool affine_est_offset,                           // 是否估计仿射偏移
    const bool affine_est_gain,                             // 是否估计仿射增益
    Keypoint* cur_px_estimate,                              // 当前像素位置估计
    double* h_inv = nullptr);                               // 可选的逆Hessian矩阵

// 用于在当前图像中进行二维对齐
bool align2D(
    const cv::Mat& cur_img,                                 
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Keypoint& cur_px_estimate,
    bool no_simd = false,                                   // 是否禁用SIMD优化
    std::vector<Eigen::Vector2f>* each_step=nullptr);       // 可选的每一步的位移向量

// align2D 的一个变种，专门为SSE2优化
bool align2D_SSE2(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,                                       // 迭代次数
    Keypoint& cur_px_estimate);    

// align2D 的一个变种，专门为NEON优化
bool align2D_NEON(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    Keypoint& cur_px_estimate);

// 用于在多层金字塔图像中进行2D对齐
void alignPyr2DVec(
    const std::vector<cv::Mat>& img_pyr_ref,                // 参考图像金字塔
    const std::vector<cv::Mat>& img_pyr_cur,                // 当前图像金字塔
    const int max_level,                                    // 金字塔的最高层级
    const int min_level,                                    // 金字塔的最低层级
    const std::vector<int>& patch_sizes,                    // 特征块的大小
    const int n_iter,                                       // 迭代次数
    const float min_update_squared,                         // 最小更新值的平方
    const std::vector<cv::Point2f>& px_ref,                 // 参考图像中的像素点
    std::vector<cv::Point2f>& px_cur,                       // 当前图像中的像素点
    std::vector<uint8_t>& status);                          // 对齐状态

bool alignPyr2D(
    const std::vector<cv::Mat>& img_pyr_ref,
    const std::vector<cv::Mat>& img_pyr_cur,
    const int max_level,
    const int min_level,
    const std::vector<int>& patch_sizes,
    const int n_iter,
    const float min_update_squared,
    const Eigen::Vector2i &px_ref_level_0,                  // 参考图像中第0层的像素点
    Keypoint &px_cur_level_0);                              // 当前图像中第0层的像素点

} // namespace feature_alignment
} // namespace svo

#endif // SVO_DIRECT_FEATURE_ALIGNMENT_H_
