#ifndef PCLEYE_H_
#define PCLEYE_H_

#include <pcl_ros/point_cloud.h>

#include "PclEyeParameters.h"
#include "PclScrew.h"
#include "PclScrewRecognitionTools.h"

class PclEye
{
    public:
    static PclEye* openUp();
    PclEye* useTheseParameters(PclEyeParameters parameters);
    std::shared_ptr<PclScrew> toFindScrewIn(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud);

    private:
    PclEye() {};
    static PclEye* m_singleton;
    PclEyeParameters m_parameters;
};

#endif