#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include "mynteye/api/api.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstring>

using std::placeholders::_1;

class DepthImuPublisher : public rclcpp::Node {
public:
  explicit DepthImuPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("depth_imu_publisher", options) {

    publish_left_   = declare_parameter<bool>("publish_left", true);
    publish_right_  = declare_parameter<bool>("publish_right", true);
    publish_depth_  = declare_parameter<bool>("publish_depth", true);
    publish_disp_   = declare_parameter<bool>("publish_disparity", false);
    publish_imu_    = declare_parameter<bool>("publish_imu", true);
    frame_prefix_   = declare_parameter<std::string>("frame_prefix", "/base_manip");
    publish_points_ = declare_parameter<bool>("publish_points", true);
    points_topic_   = declare_parameter<std::string>("points_topic", "/pointcloud_in");

    left_pub_  = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image_raw", 10);
    disp_pub_  = create_publisher<sensor_msgs::msg::Image>("disparity/image_raw", 10);
    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("imu/data", 50);

    if (publish_points_) {
        points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            points_topic_, rclcpp::SensorDataQoS());
    }    

    // Создание API как в camera_with_senior_api.cc
    int argc = 0;
    char **argv = nullptr;
    api_ = mynteye::API::Create(argc, argv);
    if (!api_) {
      throw std::runtime_error("Failed to create MYNT EYE API");
    }

    bool ok = false;
    auto request = api_->SelectStreamRequest(&ok);
    if (!ok) {
      throw std::runtime_error("Failed to select stream request");
    }
    api_->ConfigStreamRequest(request);

    // Включаем нужные потоки
    if (publish_left_)  api_->EnableStreamData(mynteye::Stream::LEFT);
    if (publish_right_) api_->EnableStreamData(mynteye::Stream::RIGHT);

    // По аналогии с sample можно также включить rectified, но оставим базовые
    api_->EnableStreamData(mynteye::Stream::LEFT_RECTIFIED);
    api_->EnableStreamData(mynteye::Stream::RIGHT_RECTIFIED);

    if (publish_disp_)  api_->EnableStreamData(mynteye::Stream::DISPARITY_NORMALIZED);
    if (publish_depth_) api_->EnableStreamData(mynteye::Stream::DEPTH);
    if (publish_points_) api_->EnableStreamData(mynteye::Stream::POINTS);

    if (publish_imu_) {
      // Важно: именно так в актуальном паттерне SDK
      api_->EnableMotionDatas();
    }

    // Callbacks потоков
    if (publish_left_) {
      api_->SetStreamCallback(
        mynteye::Stream::LEFT,
        [this](const mynteye::api::StreamData &data) {
          this->publishImage(mynteye::Stream::LEFT, data);
        }
      );
    }

    if (publish_right_) {
      api_->SetStreamCallback(
        mynteye::Stream::RIGHT,
        [this](const mynteye::api::StreamData &data) {
          this->publishImage(mynteye::Stream::RIGHT, data);
        }
      );
    }

    if (publish_disp_) {
      api_->SetStreamCallback(
        mynteye::Stream::DISPARITY_NORMALIZED,
        [this](const mynteye::api::StreamData &data) {
          this->publishImage(mynteye::Stream::DISPARITY_NORMALIZED, data);
        }
      );
    }

    if (publish_depth_) {
      api_->SetStreamCallback(
        mynteye::Stream::DEPTH,
        [this](const mynteye::api::StreamData &data) {
          this->publishImage(mynteye::Stream::DEPTH, data);
        }
      );
    }

    // IMU callback
    if (publish_imu_) {
      api_->SetMotionCallback(
        [this](const mynteye::api::MotionData &data) {
          this->publishImu(data);
        }
      );
    }

    if (publish_points_) {
      api_->SetStreamCallback(
        mynteye::Stream::POINTS,
        [this](const mynteye::api::StreamData &data) {
          this->publishPoints(data);
        }
      );
    }


    api_->Start(mynteye::Source::ALL);
  }

  ~DepthImuPublisher() override {
    try {
      if (api_) {
        api_->Stop(mynteye::Source::ALL);
      }
    } catch (...) {
      // не бросаем исключения из деструктора
    }
  }

private:
  void publishImage(const mynteye::Stream &stream, const mynteye::api::StreamData &data) {
    // В актуальном SDK кадр лежит в data.frame
    if (data.frame.empty()) {
      return;
    }

    const rclcpp::Time stamp = this->now();

    std::string frame_id = frame_prefix_;
    std::string encoding;

    // Определяем тип потока и кодировку
    if (stream == mynteye::Stream::LEFT) {
      frame_id += "/left";
      encoding = "mono8";
    } else if (stream == mynteye::Stream::RIGHT) {
      frame_id += "/right";
      encoding = "mono8";
    } else if (stream == mynteye::Stream::DISPARITY_NORMALIZED) {
      frame_id += "/disparity";
      encoding = "mono8";   // как в sample: CV_8UC1
    } else if (stream == mynteye::Stream::DEPTH) {
      frame_id += "/depth";
      encoding = "16UC1";   // как в sample: CV_16UC1
    } else {
      frame_id += "/image";
      encoding = "mono8";
    }

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;

    auto msg = cv_bridge::CvImage(header, encoding, data.frame).toImageMsg();

    if (stream == mynteye::Stream::LEFT) {
      left_pub_->publish(*msg);
    } else if (stream == mynteye::Stream::RIGHT) {
      right_pub_->publish(*msg);
    } else if (stream == mynteye::Stream::DISPARITY_NORMALIZED) {
      disp_pub_->publish(*msg);
    } else if (stream == mynteye::Stream::DEPTH) {
      depth_pub_->publish(*msg);
    }
  }

  void publishImu(const mynteye::api::MotionData &data) {
    if (!data.imu) {
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_prefix_ + "/imu";

    // Линейное ускорение
    msg.linear_acceleration.x = data.imu->accel[0];
    msg.linear_acceleration.y = data.imu->accel[1];
    msg.linear_acceleration.z = data.imu->accel[2];

    // Угловая скорость
    msg.angular_velocity.x = data.imu->gyro[0];
    msg.angular_velocity.y = data.imu->gyro[1];
    msg.angular_velocity.z = data.imu->gyro[2];

    // Ориентацию не вычисляем
    msg.orientation_covariance[0] = -1.0;

    imu_pub_->publish(msg);
  }

    void publishPoints(const mynteye::api::StreamData &data) {
        if (!publish_points_ || !points_pub_) {
        return;
        }

        // В текущем паттерне твоей ноды данные приходят в data.frame
        if (data.frame.empty()) {
        return;
        }

        const rclcpp::Time stamp = this->now();
        const std::string frame_id = "mynt_eye";

        // Ожидаемый формат CV_32FC3, как у твоего конвертера
        auto msg = matToPointCloud2(data.frame, stamp, frame_id);
        if (msg.width == 0 || msg.height == 0) {
        return;
        }

        points_pub_->publish(msg);
    }


  sensor_msgs::msg::PointCloud2
    matToPointCloud2(const cv::Mat &pts_mat,
                    const rclcpp::Time &stamp,
                    const std::string &frame_id) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    // Ожидаемый формат: CV_32FC3 (HxWx3)
    if (pts_mat.empty() || pts_mat.type() != CV_32FC3) {
        // Можно расширить под другие форматы, если выяснится иное
        msg.height = 0;
        msg.width = 0;
        return msg;
    }

    msg.height = static_cast<uint32_t>(pts_mat.rows);
    msg.width  = static_cast<uint32_t>(pts_mat.cols);
    msg.is_bigendian = false;
    msg.is_dense = false;

    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.point_step = 12;  // 3 * float32
    msg.row_step = msg.point_step * msg.width;

    const size_t total_bytes = static_cast<size_t>(msg.row_step) * msg.height;
    msg.data.resize(total_bytes);

    uint8_t *dst = msg.data.data();

    for (int r = 0; r < pts_mat.rows; ++r) {
        const auto *row_ptr = pts_mat.ptr<cv::Vec3f>(r);
        for (int c = 0; c < pts_mat.cols; ++c) {
            const cv::Vec3f &p = row_ptr[c];

            float x = p[0] / 1000.0f;
            float y = p[1] / 1000.0f;
            float z = p[2] / 1000.0f;

            std::memcpy(dst + 0, &x, sizeof(float));
            std::memcpy(dst + 4, &y, sizeof(float));
            std::memcpy(dst + 8, &z, sizeof(float));
            dst += msg.point_step;
        }
    }

    return msg;
    }

private:
  std::shared_ptr<mynteye::API> api_;

  bool publish_left_{true};
  bool publish_right_{true};
  bool publish_depth_{true};
  bool publish_disp_{false};
  bool publish_imu_{true};
  bool publish_points_{true};

  std::string points_topic_{"points"};
  std::string frame_prefix_{"mynt"};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DepthImuPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "DepthImuPublisher exception: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
