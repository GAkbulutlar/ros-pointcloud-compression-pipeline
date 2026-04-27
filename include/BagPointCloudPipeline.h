#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PipelineConfig {
    std::string bag_path = "b0-2014-07-11-11-00-49.bag";
    std::string output_bin_path = "cloud_frames.bin";
    std::size_t max_frames = 60;
    float kd_query_radius = 0.05f;
};

struct FrameMetrics {
    std::size_t frame_index = 0;
    std::size_t point_count = 0;
    std::size_t raw_size_bytes = 0;
    std::size_t compressed_size_bytes = 0;
    double capture_to_cloud_ms = 0.0;
    double compression_ms = 0.0;
    double decompression_ms = 0.0;
};

struct PipelineSummary {
    std::size_t frames_processed = 0;
    std::size_t total_raw_size_bytes = 0;
    std::size_t total_compressed_size_bytes = 0;
    double avg_capture_to_cloud_ms = 0.0;
    double avg_compression_ms = 0.0;
    double avg_decompression_ms = 0.0;
};

class BagPointCloudPipeline {
public:
    explicit BagPointCloudPipeline(PipelineConfig config);

    bool run();

private:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    bool processBag();
    void printSummary() const;
    void runKdTreeQuery(const CloudT::ConstPtr& decoded_cloud) const;
    void visualizeCloud(const CloudT::ConstPtr& cloud) const;

    static void writeFrameBlob(std::ofstream& out,
                               std::uint32_t frame_index,
                               const std::string& bytes);

    PipelineConfig config_;
    std::vector<FrameMetrics> frame_metrics_;
    PipelineSummary summary_;
    CloudT::Ptr cloud_for_viewer_;
};
