#include "BagPointCloudPipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <bzlib.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace {

using Clock = std::chrono::steady_clock;
using ByteVec = std::vector<std::uint8_t>;

struct BagRecord {
    ByteVec header;
    ByteVec data;
};

struct ConnectionInfo {
    std::string topic;
    std::string type;
};

struct ScanPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

std::string humanSize(std::size_t bytes)
{
    static const char* units[] = {"B", "KB", "MB", "GB"};
    double value = static_cast<double>(bytes);
    int unit = 0;
    while (value >= 1024.0 && unit < 3) {
        value /= 1024.0;
        ++unit;
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << value << ' ' << units[unit];
    return out.str();
}

template <typename T>
void writePod(std::ofstream& out, const T& value)
{
    out.write(reinterpret_cast<const char*>(&value), static_cast<std::streamsize>(sizeof(T)));
}

template <typename T>
bool readPod(std::istream& in, T& value)
{
    in.read(reinterpret_cast<char*>(&value), static_cast<std::streamsize>(sizeof(T)));
    return static_cast<bool>(in);
}

std::unordered_map<std::string, ByteVec> parseHeaderFields(const ByteVec& raw_header)
{
    std::unordered_map<std::string, ByteVec> fields;
    std::size_t offset = 0;

    while (offset + sizeof(std::uint32_t) <= raw_header.size()) {
        std::uint32_t field_len = 0;
        std::memcpy(&field_len, raw_header.data() + offset, sizeof(std::uint32_t));
        offset += sizeof(std::uint32_t);

        if (offset + field_len > raw_header.size()) {
            break;
        }

        const char* field_ptr = reinterpret_cast<const char*>(raw_header.data() + offset);
        std::string field(field_ptr, field_ptr + field_len);
        offset += field_len;

        const std::size_t eq = field.find('=');
        if (eq == std::string::npos) {
            continue;
        }

        const std::string key = field.substr(0, eq);
        const ByteVec value(field.begin() + static_cast<std::ptrdiff_t>(eq + 1), field.end());
        fields[key] = value;
    }

    return fields;
}

std::string fieldAsString(const std::unordered_map<std::string, ByteVec>& fields, const std::string& key)
{
    const auto it = fields.find(key);
    if (it == fields.end()) {
        return {};
    }
    return std::string(it->second.begin(), it->second.end());
}

std::uint8_t fieldAsU8(const std::unordered_map<std::string, ByteVec>& fields,
                       const std::string& key,
                       std::uint8_t fallback = 0)
{
    const auto it = fields.find(key);
    if (it == fields.end() || it->second.empty()) {
        return fallback;
    }
    return it->second[0];
}

std::uint32_t fieldAsU32(const std::unordered_map<std::string, ByteVec>& fields,
                         const std::string& key,
                         std::uint32_t fallback = 0)
{
    const auto it = fields.find(key);
    if (it == fields.end() || it->second.size() < sizeof(std::uint32_t)) {
        return fallback;
    }

    std::uint32_t out = fallback;
    std::memcpy(&out, it->second.data(), sizeof(std::uint32_t));
    return out;
}

bool readRecord(std::istream& in, BagRecord& record)
{
    std::uint32_t header_len = 0;
    if (!readPod(in, header_len)) {
        return false;
    }

    record.header.resize(header_len);
    in.read(reinterpret_cast<char*>(record.header.data()), static_cast<std::streamsize>(header_len));
    if (!in) {
        return false;
    }

    std::uint32_t data_len = 0;
    if (!readPod(in, data_len)) {
        return false;
    }

    record.data.resize(data_len);
    in.read(reinterpret_cast<char*>(record.data.data()), static_cast<std::streamsize>(data_len));
    if (!in) {
        return false;
    }

    return true;
}

bool readU32FromBuffer(const ByteVec& buf, std::size_t& offset, std::uint32_t& value)
{
    if (offset + sizeof(std::uint32_t) > buf.size()) {
        return false;
    }
    std::memcpy(&value, buf.data() + offset, sizeof(std::uint32_t));
    offset += sizeof(std::uint32_t);
    return true;
}

bool readF32FromBuffer(const ByteVec& buf, std::size_t& offset, float& value)
{
    if (offset + sizeof(float) > buf.size()) {
        return false;
    }
    std::memcpy(&value, buf.data() + offset, sizeof(float));
    offset += sizeof(float);
    return true;
}

bool skipRosString(const ByteVec& buf, std::size_t& offset)
{
    std::uint32_t len = 0;
    if (!readU32FromBuffer(buf, offset, len)) {
        return false;
    }
    if (offset + len > buf.size()) {
        return false;
    }
    offset += len;
    return true;
}

bool parseMultiEchoLaserScan(const ByteVec& payload, std::vector<ScanPoint>& points, bool vertical_topic)
{
    points.clear();

    std::size_t offset = 0;

    // std_msgs/Header
    std::uint32_t seq = 0;
    std::uint32_t stamp_sec = 0;
    std::uint32_t stamp_nsec = 0;
    if (!readU32FromBuffer(payload, offset, seq) ||
        !readU32FromBuffer(payload, offset, stamp_sec) ||
        !readU32FromBuffer(payload, offset, stamp_nsec) ||
        !skipRosString(payload, offset)) {
        return false;
    }

    float angle_min = 0.0f;
    float angle_max = 0.0f;
    float angle_increment = 0.0f;
    float time_increment = 0.0f;
    float scan_time = 0.0f;
    float range_min = 0.0f;
    float range_max = 0.0f;

    if (!readF32FromBuffer(payload, offset, angle_min) ||
        !readF32FromBuffer(payload, offset, angle_max) ||
        !readF32FromBuffer(payload, offset, angle_increment) ||
        !readF32FromBuffer(payload, offset, time_increment) ||
        !readF32FromBuffer(payload, offset, scan_time) ||
        !readF32FromBuffer(payload, offset, range_min) ||
        !readF32FromBuffer(payload, offset, range_max)) {
        return false;
    }

    std::uint32_t beam_count = 0;
    if (!readU32FromBuffer(payload, offset, beam_count)) {
        return false;
    }

    points.reserve(beam_count);
    for (std::uint32_t i = 0; i < beam_count; ++i) {
        std::uint32_t echo_count = 0;
        if (!readU32FromBuffer(payload, offset, echo_count)) {
            return false;
        }

        float selected_range = std::numeric_limits<float>::quiet_NaN();
        for (std::uint32_t e = 0; e < echo_count; ++e) {
            float r = 0.0f;
            if (!readF32FromBuffer(payload, offset, r)) {
                return false;
            }
            if (!std::isfinite(selected_range) && std::isfinite(r)) {
                selected_range = r;
            }
        }

        if (!std::isfinite(selected_range) || selected_range < range_min || selected_range > range_max) {
            continue;
        }

        const float angle = angle_min + static_cast<float>(i) * angle_increment;
        const float ca = std::cos(angle);
        const float sa = std::sin(angle);

        ScanPoint p;
        if (vertical_topic) {
            p.x = selected_range * ca;
            p.y = 0.0f;
            p.z = selected_range * sa;
        } else {
            p.x = selected_range * ca;
            p.y = selected_range * sa;
            p.z = 0.0f;
        }
        points.push_back(p);
    }

    // intensities[] (ignored)
    std::uint32_t intensities_count = 0;
    if (!readU32FromBuffer(payload, offset, intensities_count)) {
        return false;
    }
    for (std::uint32_t i = 0; i < intensities_count; ++i) {
        std::uint32_t echo_count = 0;
        if (!readU32FromBuffer(payload, offset, echo_count)) {
            return false;
        }
        const std::size_t bytes_to_skip = static_cast<std::size_t>(echo_count) * sizeof(float);
        if (offset + bytes_to_skip > payload.size()) {
            return false;
        }
        offset += bytes_to_skip;
    }

    return true;
}

bool decodeChunk(const BagRecord& chunk_record, ByteVec& decompressed)
{
    const auto header_fields = parseHeaderFields(chunk_record.header);
    const std::string compression = fieldAsString(header_fields, "compression");
    const std::uint32_t uncompressed_size = fieldAsU32(header_fields, "size", 0);

    if (compression == "none") {
        decompressed = chunk_record.data;
        return true;
    }

    if (compression != "bz2") {
        std::cerr << "Unsupported rosbag chunk compression: " << compression << "\n";
        return false;
    }

    if (uncompressed_size == 0) {
        std::cerr << "Invalid bz2 chunk size in rosbag header.\n";
        return false;
    }

    decompressed.assign(uncompressed_size, 0);
    unsigned int dst_len = uncompressed_size;
    const int bz_result = BZ2_bzBuffToBuffDecompress(
        reinterpret_cast<char*>(decompressed.data()),
        &dst_len,
        const_cast<char*>(reinterpret_cast<const char*>(chunk_record.data.data())),
        static_cast<unsigned int>(chunk_record.data.size()),
        0,
        0);

    if (bz_result != BZ_OK) {
        std::cerr << "Failed to decompress rosbag bz2 chunk (error code " << bz_result << ").\n";
        return false;
    }

    decompressed.resize(dst_len);
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr scanPointsToCloud(const std::vector<ScanPoint>& points)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(points.size());
    for (const auto& p : points) {
        cloud->push_back(pcl::PointXYZ{p.x, p.y, p.z});
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

bool topicLooksVertical(const std::string& topic)
{
    return topic.find("vertical") != std::string::npos;
}

bool typeLooksLaserScan(const std::string& type)
{
    return type.find("MultiEchoLaserScan") != std::string::npos ||
           type.find("LaserScan") != std::string::npos;
}

bool startsWithRosbagMagic(std::istream& in)
{
    std::string magic;
    std::getline(in, magic);
    return magic.rfind("#ROSBAG V2.0", 0) == 0;
}

bool processMessageDataRecord(const BagRecord& record,
                              const std::unordered_map<std::uint32_t, ConnectionInfo>& connections,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
                              std::size_t max_frames)
{
    if (clouds.size() >= max_frames) {
        return true;
    }

    const auto header_fields = parseHeaderFields(record.header);
    const std::uint32_t conn_id = fieldAsU32(header_fields, "conn", std::numeric_limits<std::uint32_t>::max());

    const auto conn_it = connections.find(conn_id);
    if (conn_it == connections.end()) {
        return true;
    }

    const ConnectionInfo& conn = conn_it->second;
    if (!typeLooksLaserScan(conn.type)) {
        return true;
    }

    std::vector<ScanPoint> scan_points;
    if (!parseMultiEchoLaserScan(record.data, scan_points, topicLooksVertical(conn.topic))) {
        return true;
    }

    auto cloud = scanPointsToCloud(scan_points);
    if (!cloud->empty()) {
        clouds.push_back(cloud);
    }

    return true;
}

bool extractLaserCloudsFromRosbag(const std::string& bag_path,
                                  std::size_t max_frames,
                                  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds)
{
    clouds.clear();

    std::ifstream in(bag_path, std::ios::binary);
    if (!in) {
        std::cerr << "Cannot open bag file: " << bag_path << "\n";
        return false;
    }

    if (!startsWithRosbagMagic(in)) {
        std::cerr << "Input file is not a ROS bag v2 file: " << bag_path << "\n";
        return false;
    }

    std::unordered_map<std::uint32_t, ConnectionInfo> connections;

    while (in && clouds.size() < max_frames) {
        BagRecord record;
        if (!readRecord(in, record)) {
            break;
        }

        const auto header_fields = parseHeaderFields(record.header);
        const std::uint8_t op = fieldAsU8(header_fields, "op", 0);

        if (op == 0x07) {
            const std::uint32_t conn_id = fieldAsU32(header_fields, "conn", std::numeric_limits<std::uint32_t>::max());
            if (conn_id == std::numeric_limits<std::uint32_t>::max()) {
                continue;
            }

            const auto conn_data_fields = parseHeaderFields(record.data);
            ConnectionInfo info;
            info.topic = fieldAsString(conn_data_fields, "topic");
            info.type = fieldAsString(conn_data_fields, "type");
            connections[conn_id] = info;
            continue;
        }

        if (op != 0x05) {
            continue;
        }

        ByteVec decompressed_chunk;
        if (!decodeChunk(record, decompressed_chunk)) {
            return false;
        }

        std::string chunk_text(reinterpret_cast<const char*>(decompressed_chunk.data()), decompressed_chunk.size());
        std::istringstream chunk_stream(chunk_text);
        while (chunk_stream && clouds.size() < max_frames) {
            BagRecord chunk_record;
            if (!readRecord(chunk_stream, chunk_record)) {
                break;
            }

            const auto chunk_header_fields = parseHeaderFields(chunk_record.header);
            const std::uint8_t chunk_op = fieldAsU8(chunk_header_fields, "op", 0);

            if (chunk_op == 0x07) {
                const std::uint32_t conn_id = fieldAsU32(chunk_header_fields, "conn", std::numeric_limits<std::uint32_t>::max());
                if (conn_id == std::numeric_limits<std::uint32_t>::max()) {
                    continue;
                }

                const auto conn_data_fields = parseHeaderFields(chunk_record.data);
                ConnectionInfo info;
                info.topic = fieldAsString(conn_data_fields, "topic");
                info.type = fieldAsString(conn_data_fields, "type");
                connections[conn_id] = info;
                continue;
            }

            if (chunk_op == 0x02) {
                if (!processMessageDataRecord(chunk_record, connections, clouds, max_frames)) {
                    return false;
                }
            }
        }
    }

    return !clouds.empty();
}

} // namespace

BagPointCloudPipeline::BagPointCloudPipeline(PipelineConfig config)
    : config_(std::move(config)), cloud_for_viewer_(new CloudT())
{
}

bool BagPointCloudPipeline::run()
{
    if (!processBag()) {
        return false;
    }

    printSummary();

    if (cloud_for_viewer_ && !cloud_for_viewer_->empty()) {
        runKdTreeQuery(cloud_for_viewer_);
        visualizeCloud(cloud_for_viewer_);
    }

    return true;
}

bool BagPointCloudPipeline::processBag()
{
    if (!std::filesystem::exists(config_.bag_path)) {
        std::cerr << "Bag file not found: " << config_.bag_path << "\n";
        return false;
    }

    std::vector<CloudT::Ptr> input_clouds;
    if (!extractLaserCloudsFromRosbag(config_.bag_path, config_.max_frames, input_clouds)) {
        std::cerr << "Could not extract laser scan frames from ROS bag: " << config_.bag_path << "\n";
        return false;
    }

    std::cout << "Loaded " << input_clouds.size() << " laser-derived frames from ROS bag.\n";

    pcl::io::OctreePointCloudCompression<PointT> encoder(
        pcl::io::MANUAL_CONFIGURATION,
        true,
        0.01,
        0.05,
        10,
        true,
        8);

    std::ofstream out(config_.output_bin_path, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::cerr << "Failed to open output file: " << config_.output_bin_path << "\n";
        return false;
    }

    for (std::size_t frame_index = 0; frame_index < input_clouds.size(); ++frame_index) {
        const auto capture_start = Clock::now();
        CloudT::Ptr cloud = input_clouds[frame_index];
        const auto capture_end = Clock::now();

        if (!cloud || cloud->empty()) {
            continue;
        }

        std::stringstream compressed_stream(std::ios::in | std::ios::out | std::ios::binary);
        const auto compress_start = Clock::now();
        encoder.encodePointCloud(cloud, compressed_stream);
        const auto compress_end = Clock::now();

        const std::string compressed_bytes = compressed_stream.str();
        writeFrameBlob(out, static_cast<std::uint32_t>(frame_index), compressed_bytes);

        std::stringstream decode_stream(compressed_bytes, std::ios::in | std::ios::binary);
        CloudT::Ptr decoded_cloud(new CloudT());
        const auto decompress_start = Clock::now();
        encoder.decodePointCloud(decode_stream, decoded_cloud);
        const auto decompress_end = Clock::now();

        FrameMetrics metrics;
        metrics.frame_index = frame_index;
        metrics.point_count = cloud->size();
        metrics.raw_size_bytes = cloud->size() * sizeof(PointT);
        metrics.compressed_size_bytes = compressed_bytes.size();
        metrics.capture_to_cloud_ms =
            std::chrono::duration<double, std::milli>(capture_end - capture_start).count();
        metrics.compression_ms =
            std::chrono::duration<double, std::milli>(compress_end - compress_start).count();
        metrics.decompression_ms =
            std::chrono::duration<double, std::milli>(decompress_end - decompress_start).count();

        frame_metrics_.push_back(metrics);

        summary_.frames_processed += 1;
        summary_.total_raw_size_bytes += metrics.raw_size_bytes;
        summary_.total_compressed_size_bytes += metrics.compressed_size_bytes;
        summary_.avg_capture_to_cloud_ms += metrics.capture_to_cloud_ms;
        summary_.avg_compression_ms += metrics.compression_ms;
        summary_.avg_decompression_ms += metrics.decompression_ms;

        cloud_for_viewer_ = decoded_cloud;

        std::cout << "Frame " << frame_index
                  << " points=" << metrics.point_count
                  << " raw=" << humanSize(metrics.raw_size_bytes)
                  << " compressed=" << humanSize(metrics.compressed_size_bytes)
                  << " capture_ms=" << std::fixed << std::setprecision(3)
                  << metrics.capture_to_cloud_ms
                  << " comp_ms=" << metrics.compression_ms
                  << " decomp_ms=" << metrics.decompression_ms
                  << "\n";
    }

    if (summary_.frames_processed == 0) {
        std::cerr << "No frames were processed. Check ROS bag topics and message types.\n";
        return false;
    }

    const double count = static_cast<double>(summary_.frames_processed);
    summary_.avg_capture_to_cloud_ms /= count;
    summary_.avg_compression_ms /= count;
    summary_.avg_decompression_ms /= count;

    return true;
}

void BagPointCloudPipeline::printSummary() const
{
    const double compression_ratio = summary_.total_compressed_size_bytes == 0
                                         ? 0.0
                                         : static_cast<double>(summary_.total_raw_size_bytes) /
                                               static_cast<double>(summary_.total_compressed_size_bytes);

    std::cout << "\n===== Pipeline Summary =====\n";
    std::cout << "Bag file: " << config_.bag_path << "\n";
    std::cout << "Frames processed: " << summary_.frames_processed << "\n";
    std::cout << "Raw total: " << summary_.total_raw_size_bytes
              << " bytes (" << humanSize(summary_.total_raw_size_bytes) << ")\n";
    std::cout << "Compressed total: " << summary_.total_compressed_size_bytes
              << " bytes (" << humanSize(summary_.total_compressed_size_bytes) << ")\n";
    std::cout << "Compression ratio (raw/compressed): " << std::fixed << std::setprecision(3)
              << compression_ratio << "x\n";
    std::cout << "Avg capture->cloud latency: " << summary_.avg_capture_to_cloud_ms << " ms\n";
    std::cout << "Avg compression latency: " << summary_.avg_compression_ms << " ms\n";
    std::cout << "Avg decompression latency: " << summary_.avg_decompression_ms << " ms\n";
    std::cout << "Compressed stream written to: " << config_.output_bin_path << "\n";
}

void BagPointCloudPipeline::runKdTreeQuery(const CloudT::ConstPtr& decoded_cloud) const
{
    if (!decoded_cloud || decoded_cloud->empty()) {
        return;
    }

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(decoded_cloud);

    const PointT query = decoded_cloud->points.front();

    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_dist_sq;

    const int found = kdtree.radiusSearch(query,
                                          config_.kd_query_radius,
                                          neighbor_indices,
                                          neighbor_dist_sq);

    std::cout << "\n===== k-d Tree Query =====\n";
    std::cout << "Query point: [" << query.x << ", " << query.y << ", " << query.z << "]\n";
    std::cout << "Radius: " << config_.kd_query_radius << " m\n";
    std::cout << "Neighbors found: " << found << "\n";

    if (!neighbor_dist_sq.empty()) {
        std::cout << "Closest neighbor distance: "
                  << std::sqrt(std::max(0.0f, neighbor_dist_sq.front())) << " m\n";
    }
}

void BagPointCloudPipeline::visualizeCloud(const CloudT::ConstPtr& cloud) const
{
    pcl::visualization::PCLVisualizer viewer("Decoded Point Cloud");
    viewer.setBackgroundColor(0.05, 0.05, 0.08);
    viewer.addCoordinateSystem(0.2);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cyan(cloud, 80, 220, 255);
    viewer.addPointCloud<PointT>(cloud, cyan, "decoded_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            2,
                                            "decoded_cloud");
    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spinOnce(16);
    }
}

void BagPointCloudPipeline::writeFrameBlob(std::ofstream& out,
                                           std::uint32_t frame_index,
                                           const std::string& bytes)
{
    const std::uint64_t payload_size = static_cast<std::uint64_t>(bytes.size());
    writePod(out, frame_index);
    writePod(out, payload_size);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}
