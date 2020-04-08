#ifndef PCLSCREW_H_
#define PCLSCREW_H_

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

class PclScrew
{
    public:
    PclScrew(pcl::PointCloud<pcl::PointXYZ> pCloud,
             pcl::PointXYZ pointMin, 
             pcl::PointXYZ pointMax, 
             Eigen::Quaternionf rotation, 
             Eigen::Vector3f translation) 
             : m_pCloud(pCloud),
               m_pointMin(pointMin),
               m_pointMax(pointMax),
               m_rotation(rotation),
               m_translation(translation) {};
    pcl::PointCloud<pcl::PointXYZ> getPCloud() { return m_pCloud; };

    private:
    pcl::PointCloud<pcl::PointXYZ> m_pCloud;
    pcl::PointXYZ m_pointMin;
    pcl::PointXYZ m_pointMax;
    Eigen::Quaternionf m_rotation;
    Eigen::Vector3f m_translation;
};

#endif