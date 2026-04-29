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
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <bzlib.h>
#include <lz4frame.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/voxel_grid.h>
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

struct PointFieldInfo {
    std::string name;
    std::uint32_t offset = 0;
    std::uint8_t datatype = 0;
    std::uint32_t count = 0;
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

double bytesToMb(std::size_t bytes)
{
    return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

double safeRatio(std::size_t raw_bytes, std::size_t compressed_bytes)
{
    if (compressed_bytes == 0) {
        return 0.0;
    }
    return static_cast<double>(raw_bytes) / static_cast<double>(compressed_bytes);
}

bool serializeCloudRaw(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string& bytes)
{
    if (!cloud) {
        return false;
    }

    const std::size_t payload_size = cloud->size() * sizeof(pcl::PointXYZ);
    bytes.resize(payload_size);
    if (payload_size > 0) {
        std::memcpy(bytes.data(), cloud->points.data(), payload_size);
    }
    return true;
}

bool deserializeCloudRaw(const std::string& bytes, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (bytes.size() % sizeof(pcl::PointXYZ) != 0) {
        return false;
    }

    const std::size_t point_count = bytes.size() / sizeof(pcl::PointXYZ);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(point_count);
    if (!bytes.empty()) {
        std::memcpy(cloud->points.data(), bytes.data(), bytes.size());
    }
    cloud->width = static_cast<std::uint32_t>(point_count);
    cloud->height = 1;
    cloud->is_dense = false;
    return true;
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

bool readU8FromBuffer(const ByteVec& buf, std::size_t& offset, std::uint8_t& value)
{
    if (offset + sizeof(std::uint8_t) > buf.size()) {
        return false;
    }
    value = buf[offset];
    offset += sizeof(std::uint8_t);
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

    if (uncompressed_size == 0) {
        std::cerr << "Invalid chunk size in rosbag header for compression mode: "
                  << compression << "\n";
        return false;
    }

    if (compression == "lz4") {
        decompressed.assign(uncompressed_size, 0);
        LZ4F_dctx* context = nullptr;
        const size_t create_result = LZ4F_createDecompressionContext(&context, LZ4F_VERSION);
        if (LZ4F_isError(create_result)) {
            std::cerr << "Failed to initialize lz4 decompressor: "
                      << LZ4F_getErrorName(create_result) << "\n";
            return false;
        }

        const std::uint8_t* src = chunk_record.data.data();
        size_t src_remaining = chunk_record.data.size();
        size_t dst_offset = 0;

        while (src_remaining > 0 && dst_offset < uncompressed_size) {
            size_t src_size = src_remaining;
            size_t dst_size = uncompressed_size - dst_offset;

            const size_t result = LZ4F_decompress(
                context,
                decompressed.data() + dst_offset,
                &dst_size,
                src,
                &src_size,
                nullptr);

            if (LZ4F_isError(result)) {
                LZ4F_freeDecompressionContext(context);
                std::cerr << "Failed to decompress rosbag lz4 chunk: "
                          << LZ4F_getErrorName(result) << "\n";
                return false;
            }

            src += src_size;
            src_remaining -= src_size;
            dst_offset += dst_size;

            if (result == 0) {
                break;
            }

            if (src_size == 0 && dst_size == 0) {
                LZ4F_freeDecompressionContext(context);
                std::cerr << "Failed to decompress rosbag lz4 chunk: decoder made no progress.\n";
                return false;
            }
        }

        LZ4F_freeDecompressionContext(context);
        if (dst_offset != uncompressed_size) {
            std::cerr << "Failed to decompress rosbag lz4 chunk: size mismatch.\n";
            return false;
        }

        decompressed.resize(dst_offset);
        return true;
    }

    if (compression != "bz2") {
        std::cerr << "Unsupported rosbag chunk compression: " << compression << "\n";
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

bool typeLooksPointCloud2(const std::string& type)
{
    return type.find("PointCloud2") != std::string::npos;
}

bool readRosString(const ByteVec& buf, std::size_t& offset, std::string& value)
{
    std::uint32_t len = 0;
    if (!readU32FromBuffer(buf, offset, len)) {
        return false;
    }
    if (offset + len > buf.size()) {
        return false;
    }
    value.assign(reinterpret_cast<const char*>(buf.data() + offset), len);
    offset += len;
    return true;
}

float readTypedFieldAsFloat(const std::uint8_t* field_ptr, std::uint8_t datatype)
{
    switch (datatype) {
    case 7: {
        float value = 0.0f;
        std::memcpy(&value, field_ptr, sizeof(float));
        return value;
    }
    case 8: {
        double value = 0.0;
        std::memcpy(&value, field_ptr, sizeof(double));
        return static_cast<float>(value);
    }
    default:
        return std::numeric_limits<float>::quiet_NaN();
    }
}

bool parsePointCloud2(const ByteVec& payload, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::size_t offset = 0;

    std::uint32_t seq = 0;
    std::uint32_t stamp_sec = 0;
    std::uint32_t stamp_nsec = 0;
    std::string frame_id;
    if (!readU32FromBuffer(payload, offset, seq) ||
        !readU32FromBuffer(payload, offset, stamp_sec) ||
        !readU32FromBuffer(payload, offset, stamp_nsec) ||
        !readRosString(payload, offset, frame_id)) {
        return false;
    }

    std::uint32_t height = 0;
    std::uint32_t width = 0;
    if (!readU32FromBuffer(payload, offset, height) ||
        !readU32FromBuffer(payload, offset, width)) {
        return false;
    }

    std::uint32_t field_count = 0;
    if (!readU32FromBuffer(payload, offset, field_count)) {
        return false;
    }

    std::vector<PointFieldInfo> fields;
    fields.reserve(field_count);
    for (std::uint32_t index = 0; index < field_count; ++index) {
        PointFieldInfo field;
        if (!readRosString(payload, offset, field.name) ||
            !readU32FromBuffer(payload, offset, field.offset) ||
            !readU8FromBuffer(payload, offset, field.datatype) ||
            !readU32FromBuffer(payload, offset, field.count)) {
            return false;
        }
        fields.push_back(field);
    }

    std::uint8_t is_bigendian = 0;
    std::uint32_t point_step = 0;
    std::uint32_t row_step = 0;
    std::uint32_t data_size = 0;
    if (!readU8FromBuffer(payload, offset, is_bigendian) ||
        !readU32FromBuffer(payload, offset, point_step) ||
        !readU32FromBuffer(payload, offset, row_step) ||
        !readU32FromBuffer(payload, offset, data_size)) {
        return false;
    }

    if (is_bigendian != 0) {
        return false;
    }

    if (offset + data_size > payload.size()) {
        return false;
    }

    const std::uint8_t* data = payload.data() + offset;
    offset += data_size;

    std::uint8_t is_dense = 0;
    if (!readU8FromBuffer(payload, offset, is_dense)) {
        return false;
    }

    const PointFieldInfo* x_field = nullptr;
    const PointFieldInfo* y_field = nullptr;
    const PointFieldInfo* z_field = nullptr;
    for (const auto& field : fields) {
        if (field.name == "x") {
            x_field = &field;
        } else if (field.name == "y") {
            y_field = &field;
        } else if (field.name == "z") {
            z_field = &field;
        }
    }

    if (!x_field || !y_field || !z_field || point_step == 0) {
        return false;
    }

    const std::size_t point_count = static_cast<std::size_t>(height) * static_cast<std::size_t>(width);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(point_count);

    for (std::size_t index = 0; index < point_count; ++index) {
        const std::size_t base = index * static_cast<std::size_t>(point_step);
        if (base + point_step > data_size) {
            break;
        }

        const float x = readTypedFieldAsFloat(data + base + x_field->offset, x_field->datatype);
        const float y = readTypedFieldAsFloat(data + base + y_field->offset, y_field->datatype);
        const float z = readTypedFieldAsFloat(data + base + z_field->offset, z_field->datatype);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        cloud->push_back(pcl::PointXYZ{x, y, z});
    }

    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = is_dense != 0;
    return !cloud->empty();
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    if (typeLooksLaserScan(conn.type)) {
        std::vector<ScanPoint> scan_points;
        if (!parseMultiEchoLaserScan(record.data, scan_points, topicLooksVertical(conn.topic))) {
            return true;
        }
        cloud = scanPointsToCloud(scan_points);
    } else if (typeLooksPointCloud2(conn.type)) {
        if (!parsePointCloud2(record.data, cloud)) {
            return true;
        }
    } else {
        return true;
    }

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

    if (config_.enable_visualization && cloud_for_viewer_ && !cloud_for_viewer_->empty()) {
        runKdTreeQuery(cloud_for_viewer_);
        visualizeCloud(cloud_for_viewer_);
    } else if (cloud_for_viewer_ && !cloud_for_viewer_->empty()) {
        runKdTreeQuery(cloud_for_viewer_);
    }

    return true;
}

bool BagPointCloudPipeline::processBag()
{
    frame_metrics_.clear();
    summaries_.clear();

    if (!std::filesystem::exists(config_.bag_path)) {
        std::cerr << "Bag file not found: " << config_.bag_path << "\n";
        return false;
    }

    std::vector<CloudT::Ptr> input_clouds;
    if (!extractLaserCloudsFromRosbag(config_.bag_path, config_.max_frames, input_clouds)) {
        std::cerr << "Could not extract point cloud frames from ROS bag: " << config_.bag_path << "\n";
        return false;
    }

    std::cout << "Loaded " << input_clouds.size() << " point cloud frames from ROS bag.\n";

    struct MethodContext {
        std::string name;
        std::ofstream out;
    };

    const std::filesystem::path out_path(config_.output_bin_path);
    const std::string stem = out_path.stem().string();
    const std::string ext = out_path.extension().string().empty() ? ".bin" : out_path.extension().string();
    const std::filesystem::path parent = out_path.parent_path();

    std::vector<MethodContext> methods;
    methods.reserve(3);
    methods.push_back(MethodContext{"Raw", std::ofstream(parent / (stem + "_raw" + ext), std::ios::binary | std::ios::trunc)});
    methods.push_back(MethodContext{"Octree", std::ofstream(parent / (stem + "_octree" + ext), std::ios::binary | std::ios::trunc)});
    methods.push_back(MethodContext{"VoxelGrid", std::ofstream(parent / (stem + "_voxel" + ext), std::ios::binary | std::ios::trunc)});

    for (auto& method : methods) {
        if (!method.out) {
            std::cerr << "Failed to open output stream for method: " << method.name << "\n";
            return false;
        }
    }

    std::map<std::string, std::size_t> summary_index;
    for (const auto& method : methods) {
        PipelineSummary s;
        s.method = method.name;
        summary_index[method.name] = summaries_.size();
        summaries_.push_back(s);
    }

    pcl::io::OctreePointCloudCompression<PointT> octree_encoder(
        pcl::io::MANUAL_CONFIGURATION,
        true,
        0.01,
        0.05,
        10,
        true,
        8);

    for (std::size_t frame_index = 0; frame_index < input_clouds.size(); ++frame_index) {
        const auto capture_start = Clock::now();
        CloudT::Ptr cloud = input_clouds[frame_index];
        const auto capture_end = Clock::now();

        if (!cloud || cloud->empty()) {
            continue;
        }

        const std::size_t raw_size_bytes = cloud->size() * sizeof(PointT);

        for (auto& method : methods) {
            std::string encoded_bytes;
            CloudT::Ptr decoded_cloud(new CloudT());

            const auto compress_start = Clock::now();
            if (method.name == "Raw") {
                if (!serializeCloudRaw(cloud, encoded_bytes)) {
                    std::cerr << "Raw serialization failed on frame " << frame_index << "\n";
                    return false;
                }
            } else if (method.name == "Octree") {
                std::stringstream compressed_stream(std::ios::in | std::ios::out | std::ios::binary);
                octree_encoder.encodePointCloud(cloud, compressed_stream);
                encoded_bytes = compressed_stream.str();
            } else if (method.name == "VoxelGrid") {
                pcl::VoxelGrid<PointT> voxel_filter;
                voxel_filter.setInputCloud(cloud);
                voxel_filter.setLeafSize(config_.voxel_leaf_size,
                                         config_.voxel_leaf_size,
                                         config_.voxel_leaf_size);
                CloudT::Ptr filtered(new CloudT());
                voxel_filter.filter(*filtered);
                if (!serializeCloudRaw(filtered, encoded_bytes)) {
                    std::cerr << "VoxelGrid serialization failed on frame " << frame_index << "\n";
                    return false;
                }
            }
            const auto compress_end = Clock::now();

            writeFrameBlob(method.out, static_cast<std::uint32_t>(frame_index), encoded_bytes);

            const auto decompress_start = Clock::now();
            if (method.name == "Raw" || method.name == "VoxelGrid") {
                if (!deserializeCloudRaw(encoded_bytes, decoded_cloud)) {
                    std::cerr << "Raw deserialize failed for method " << method.name
                              << " on frame " << frame_index << "\n";
                    return false;
                }
            } else if (method.name == "Octree") {
                std::stringstream decode_stream(encoded_bytes, std::ios::in | std::ios::binary);
                octree_encoder.decodePointCloud(decode_stream, decoded_cloud);
            }
            const auto decompress_end = Clock::now();

            FrameMetrics metrics;
            metrics.method = method.name;
            metrics.frame_index = frame_index;
            metrics.point_count = cloud->size();
            metrics.raw_size_bytes = raw_size_bytes;
            metrics.compressed_size_bytes = encoded_bytes.size();
            metrics.capture_to_cloud_ms =
                std::chrono::duration<double, std::milli>(capture_end - capture_start).count();
            metrics.compression_ms =
                std::chrono::duration<double, std::milli>(compress_end - compress_start).count();
            metrics.decompression_ms =
                std::chrono::duration<double, std::milli>(decompress_end - decompress_start).count();
            frame_metrics_.push_back(metrics);

            auto& summary = summaries_[summary_index[method.name]];
            summary.frames_processed += 1;
            summary.total_raw_size_bytes += metrics.raw_size_bytes;
            summary.total_compressed_size_bytes += metrics.compressed_size_bytes;
            summary.avg_capture_to_cloud_ms += metrics.capture_to_cloud_ms;
            summary.avg_compression_ms += metrics.compression_ms;
            summary.avg_decompression_ms += metrics.decompression_ms;

            if (method.name == "Octree") {
                cloud_for_viewer_ = decoded_cloud;
            }
        }
    }

    if (summaries_.empty() || summaries_.front().frames_processed == 0) {
        std::cerr << "No frames were processed. Check ROS bag topics and message types.\n";
        return false;
    }

    for (auto& summary : summaries_) {
        if (summary.frames_processed == 0) {
            continue;
        }
        const double count = static_cast<double>(summary.frames_processed);
        summary.avg_capture_to_cloud_ms /= count;
        summary.avg_compression_ms /= count;
        summary.avg_decompression_ms /= count;
    }

    if (!writeMetricArtifacts()) {
        return false;
    }

    return true;
}

void BagPointCloudPipeline::printSummary() const
{
    std::cout << "\n===== Pipeline Summary =====\n";
    std::cout << "Bag file: " << config_.bag_path << "\n";
    if (!summaries_.empty()) {
        std::cout << "Frames processed: " << summaries_.front().frames_processed << "\n";
    }

    std::cout << "\nMethod        Size(MB)   Ratio   Comp(ms)   Decomp(ms)\n";
    std::cout << "-----------------------------------------------------\n";

    for (const auto& summary : summaries_) {
        const double ratio = safeRatio(summary.total_raw_size_bytes, summary.total_compressed_size_bytes);
        std::cout << std::left << std::setw(12) << summary.method
                  << std::right << std::setw(9) << std::fixed << std::setprecision(2)
                  << bytesToMb(summary.total_compressed_size_bytes)
                  << std::setw(8) << std::setprecision(2) << ratio
                  << std::setw(11) << std::setprecision(3) << summary.avg_compression_ms
                  << std::setw(13) << std::setprecision(3) << summary.avg_decompression_ms
                  << "\n";
    }

    std::cout << "\nPer-method frame blobs written with prefix: " << config_.output_bin_path << "\n";
    std::cout << "Per-frame metrics CSV: "
              << (std::filesystem::path(config_.output_bin_path).parent_path() /
                  (std::filesystem::path(config_.output_bin_path).stem().string() + "_metrics.csv"))
              << "\n";
    std::cout << "Method summary CSV: "
              << (std::filesystem::path(config_.output_bin_path).parent_path() /
                  (std::filesystem::path(config_.output_bin_path).stem().string() + "_summary.csv"))
              << "\n";
}

bool BagPointCloudPipeline::writeMetricArtifacts() const
{
    const std::filesystem::path out_path(config_.output_bin_path);
    const std::filesystem::path parent = out_path.parent_path();
    const std::string stem = out_path.stem().string();

    const std::filesystem::path metrics_csv = parent / (stem + "_metrics.csv");
    const std::filesystem::path summary_csv = parent / (stem + "_summary.csv");
    if (!writeMetricsCsv(metrics_csv.string()) || !writeMethodSummaryCsv(summary_csv.string())) {
        return false;
    }

    if (!config_.enable_metric_plots) {
        return true;
    }

    const std::filesystem::path ratio_svg = parent / (stem + "_ratio_vs_frame.svg");
    const std::filesystem::path latency_svg = parent / (stem + "_latency_vs_frame.svg");

    return writeLineChartSvg(ratio_svg.string(),
                             "Compression Ratio vs Frame Index",
                             "Compression Ratio (raw/compressed)",
                             false) &&
           writeLineChartSvg(latency_svg.string(),
                             "Latency vs Frame Index",
                             "Latency (comp + decomp) ms",
                             true);
}

bool BagPointCloudPipeline::writeMetricsCsv(const std::string& path) const
{
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        std::cerr << "Failed to write metrics CSV: " << path << "\n";
        return false;
    }

    out << "frame,method,point_count,raw_size_bytes,compressed_size_bytes,ratio,compression_ms,decompression_ms,total_latency_ms\n";
    for (const auto& metrics : frame_metrics_) {
        const double ratio = safeRatio(metrics.raw_size_bytes, metrics.compressed_size_bytes);
        const double total_latency = metrics.compression_ms + metrics.decompression_ms;
        out << metrics.frame_index << ','
            << metrics.method << ','
            << metrics.point_count << ','
            << metrics.raw_size_bytes << ','
            << metrics.compressed_size_bytes << ','
            << std::fixed << std::setprecision(6) << ratio << ','
            << metrics.compression_ms << ','
            << metrics.decompression_ms << ','
            << total_latency << '\n';
    }

    return true;
}

bool BagPointCloudPipeline::writeMethodSummaryCsv(const std::string& path) const
{
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        std::cerr << "Failed to write summary CSV: " << path << "\n";
        return false;
    }

    out << "method,frames,size_mb,ratio,avg_comp_ms,avg_decomp_ms\n";
    for (const auto& summary : summaries_) {
        out << summary.method << ','
            << summary.frames_processed << ','
            << std::fixed << std::setprecision(6) << bytesToMb(summary.total_compressed_size_bytes) << ','
            << safeRatio(summary.total_raw_size_bytes, summary.total_compressed_size_bytes) << ','
            << summary.avg_compression_ms << ','
            << summary.avg_decompression_ms << '\n';
    }
    return true;
}

bool BagPointCloudPipeline::writeLineChartSvg(const std::string& path,
                                              const std::string& title,
                                              const std::string& y_label,
                                              bool use_latency_metric) const
{
    std::map<std::string, std::vector<std::pair<double, double>>> series;
    std::size_t max_frame_index = 0;
    double max_y = 0.0;

    for (const auto& metrics : frame_metrics_) {
        const double y = use_latency_metric
                             ? (metrics.compression_ms + metrics.decompression_ms)
                             : safeRatio(metrics.raw_size_bytes, metrics.compressed_size_bytes);
        series[metrics.method].push_back({static_cast<double>(metrics.frame_index), y});
        max_frame_index = std::max(max_frame_index, metrics.frame_index);
        max_y = std::max(max_y, y);
    }

    if (series.empty()) {
        return true;
    }

    if (max_y <= 0.0) {
        max_y = 1.0;
    }

    const int width = 1000;
    const int height = 500;
    const int left = 80;
    const int right = 40;
    const int top = 60;
    const int bottom = 70;
    const double plot_w = static_cast<double>(width - left - right);
    const double plot_h = static_cast<double>(height - top - bottom);

    auto xToPx = [&](double x) {
        if (max_frame_index == 0) {
            return static_cast<double>(left);
        }
        return static_cast<double>(left) + (x / static_cast<double>(max_frame_index)) * plot_w;
    };
    auto yToPx = [&](double y) {
        return static_cast<double>(top) + (1.0 - (y / max_y)) * plot_h;
    };

    const std::vector<std::string> colors = {"#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd"};
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        std::cerr << "Failed to write plot SVG: " << path << "\n";
        return false;
    }

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    out << "  <rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    out << "  <text x=\"" << (width / 2) << "\" y=\"32\" text-anchor=\"middle\" font-size=\"22\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#111\">"
        << title << "</text>\n";

    out << "  <line x1=\"" << left << "\" y1=\"" << (height - bottom)
        << "\" x2=\"" << (width - right) << "\" y2=\"" << (height - bottom)
        << "\" stroke=\"#222\" stroke-width=\"2\"/>\n";
    out << "  <line x1=\"" << left << "\" y1=\"" << top
        << "\" x2=\"" << left << "\" y2=\"" << (height - bottom)
        << "\" stroke=\"#222\" stroke-width=\"2\"/>\n";

    for (int i = 0; i <= 5; ++i) {
        const double t = static_cast<double>(i) / 5.0;
        const double y_val = max_y * t;
        const double y = yToPx(y_val);
        out << "  <line x1=\"" << left << "\" y1=\"" << y
            << "\" x2=\"" << (width - right) << "\" y2=\"" << y
            << "\" stroke=\"#e7e7e7\" stroke-width=\"1\"/>\n";
        out << "  <text x=\"" << (left - 8) << "\" y=\"" << (y + 4)
            << "\" text-anchor=\"end\" font-size=\"11\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#333\">"
            << std::fixed << std::setprecision(2) << y_val << "</text>\n";
    }

    for (int i = 0; i <= 10; ++i) {
        const double t = static_cast<double>(i) / 10.0;
        const double frame = static_cast<double>(max_frame_index) * t;
        const double x = xToPx(frame);
        out << "  <line x1=\"" << x << "\" y1=\"" << top
            << "\" x2=\"" << x << "\" y2=\"" << (height - bottom)
            << "\" stroke=\"#f2f2f2\" stroke-width=\"1\"/>\n";
        out << "  <text x=\"" << x << "\" y=\"" << (height - bottom + 22)
            << "\" text-anchor=\"middle\" font-size=\"11\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#333\">"
            << std::fixed << std::setprecision(0) << frame << "</text>\n";
    }

    std::size_t color_index = 0;
    std::size_t legend_row = 0;
    for (const auto& kv : series) {
        const std::string& method = kv.first;
        const auto& points = kv.second;
        const std::string color = colors[color_index % colors.size()];
        color_index += 1;

        out << "  <polyline fill=\"none\" stroke=\"" << color
            << "\" stroke-width=\"2.5\" points=\"";
        for (const auto& pt : points) {
            out << std::fixed << std::setprecision(2) << xToPx(pt.first) << ',' << yToPx(pt.second) << ' ';
        }
        out << "\"/>\n";

        const int legend_x = left + static_cast<int>(legend_row) * 200;
        const int legend_y = height - 20;
        out << "  <line x1=\"" << legend_x << "\" y1=\"" << legend_y
            << "\" x2=\"" << (legend_x + 20) << "\" y2=\"" << legend_y
            << "\" stroke=\"" << color << "\" stroke-width=\"3\"/>\n";
        out << "  <text x=\"" << (legend_x + 26) << "\" y=\"" << (legend_y + 4)
            << "\" font-size=\"12\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#111\">"
            << method << "</text>\n";
        legend_row += 1;
    }

    out << "  <text x=\"" << (left + static_cast<int>(plot_w / 2.0))
        << "\" y=\"" << (height - 40)
        << "\" text-anchor=\"middle\" font-size=\"13\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#111\">"
        << "Frame Index" << "</text>\n";

    out << "  <text transform=\"translate(20 " << (top + static_cast<int>(plot_h / 2.0))
        << ") rotate(-90)\" text-anchor=\"middle\" font-size=\"13\" font-family=\"Segoe UI, Arial, sans-serif\" fill=\"#111\">"
        << y_label << "</text>\n";

    out << "</svg>\n";
    return true;
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
