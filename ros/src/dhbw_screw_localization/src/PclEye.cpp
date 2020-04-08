#include "dhbw_screw_localization/PclEye.h"

PclEye* PclEye::m_singleton = nullptr;

PclEye* PclEye::openUp()
{
    //Disable PCL warnings ...
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
    if(!m_singleton)
        m_singleton = new PclEye();
    
    return m_singleton;
}

PclEye* PclEye::useTheseParameters(PclEyeParameters parameters)
{
    m_parameters = parameters;
    return m_singleton;
}

PclScrewPtr PclEye::toFindScrewIn(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud)
{
    return PclScrewRecognitionTools(m_parameters).localizeScrew(pCloud);
}
