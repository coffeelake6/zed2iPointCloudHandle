#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
// #include <pcl/gpu/segmentation/gpu_sac_segmentation.h>



//点云滤波函数
void filters(
  pcl::PointCloud<pcl::PointXYZRGB> &cloud, //点云原始数据
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ground,  // 输出的地面点云 (暂时用不到)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cones //输出的非地面点云
  )
{
  // cloud_ground->clear();
  // cloud_cones->clear();
  //cloud_ground->reserve(cloud.size()/4);
  // cloud_cones->reserve(cloud.size());

  pcl::PointCloud<pcl::PointXYZRGB> filtered;  // 过滤后的点云
  
  //实车代码中的手动过滤点云
  for(auto &iter : cloud.points)
    {
      if(std::abs(iter.y) > 3 || std::hypot(iter.x , iter.y) < sqrt(1) || iter.z > 1.4 || iter.x < 0.3 || iter.x > 50)
      {
        continue;
      }
      filtered.push_back(iter);
      //cloud_cones->push_back(iter);
    }

  // pcl::PointCloud<pcl::PointXYZRGB> voxeled;
  // //pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxeled(new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  // sor.setInputCloud(filtered.makeShared());
  // sor.setLeafSize(0.06f, 0.06f, 0.06f);
  // sor.filter(voxeled);



  pcl::ModelCoefficients coefficients;  // 平面模型系数
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  // 内点索引
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;  // 平面分割对象
  seg.setOptimizeCoefficients(true);  // 优化模型系数
  seg.setModelType(pcl::SACMODEL_PLANE);  // 设置模型类型为平面
  seg.setMethodType(pcl::SAC_RANSAC);  // 设置方法类型为RANSAC
  seg.setMaxIterations(25);  // 设置最大迭代次数
  seg.setDistanceThreshold(0.02);  // 设置距离阈值  地面不平整设大点
  seg.setInputCloud(filtered.makeShared());  // 设置输入点云
  seg.segment(*inliers, coefficients);  // 执行分割，获取内点和模型系数

  if (inliers->indices.empty()) {
      ROS_WARN("Plane fitting failed: No inliers found.");  // 警告：未找到内点
      return;
  }


  pcl::ExtractIndices<pcl::PointXYZRGB> extract;  // 索引提取对象
  extract.setInputCloud(filtered.makeShared());  // 设置输入点云
  extract.setIndices(inliers);  // 设置内点索引
  extract.filter(*cloud_ground);  // 提取地面点云

  extract.setNegative(true);  // 设置为提取非内点
  extract.filter(*cloud_cones);  // 提取非地面点云


  // 将非地面点的z坐标设置为0
  // for (auto &point : cloud_cones->points)
  // {
  //   point.z = 0;
  // }
      

  ROS_INFO("cloud_cones: %ld data points", cloud_cones->size());
  // ROS_INFO("cloud_ground: %ld data points", cloud_ground->size());

      
}

//回调函数  
void CallBack(
  const sensor_msgs::PointCloud2ConstPtr &msg,
  ros::Publisher &pub_ground,
  ros::Publisher &pub_cones)
{
  ros::Time start_time = ros::Time::now();  // 记录开始时间
  
  pcl::PointCloud<pcl::PointXYZRGB> raw;  // 原始点云
  pcl::fromROSMsg(*msg, raw);  // 将ROS消息转换为PCL点云

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);  // 地面点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cones(new pcl::PointCloud<pcl::PointXYZRGB>);  // 非地面点云
  
  //调用点云处理函数(后面的数据就是滤波完的数据)
  // pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
  filters(raw,cloud_ground,cloud_cones);

  
  //发布处理后的点云
  sensor_msgs::PointCloud2 output_ground;
  sensor_msgs::PointCloud2 output_cones;
  //pcl转换为ros消息
  pcl::toROSMsg(*cloud_cones , output_cones);
  pcl::toROSMsg(*cloud_ground , output_ground);
  output_cones.header.frame_id = "base_link";
  output_ground.header.frame_id = "base_link";
  
  pub_cones.publish(output_cones);
  pub_ground.publish(output_ground);

  ros::Time end_time = ros::Time::now();  // 记录结束时间
  ROS_INFO("Preprocessing time: %.6f seconds", (end_time - start_time).toSec());  // 输出预处理时间
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filters");
  ros::NodeHandle nh;

  ros::Publisher pub_cones = nh.advertise<sensor_msgs::PointCloud2>("filters_cones", 10);
  ros::Publisher pub_ground = nh.advertise<sensor_msgs::PointCloud2>("filters_ground", 10);
  //订阅相机的点云数据
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed2i/zed_node/point_cloud/cloud_registered",1,boost::bind(&CallBack,_1,pub_ground,pub_cones));

  ros::spin();
  return 0;
}








