#ifndef PCLSCREWRECOGNITIONTOOLS_H_H
#define PCLSCREWRECOGNITIONTOOLS_H_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ml/svm_wrapper.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

#include "dhbw_screw_localization/PclEye.h"

#include <vector>
#include <memory>

typedef pcl::PointCloud<pcl::PointXYZ> PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudPtr;
typedef std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PCloudPtrVector;
typedef std::shared_ptr<PclScrew> PclScrewPtr;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHFeature;
typedef pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHFeaturePtr;
typedef pcl::SVMData PclSvmData;
typedef std::vector<pcl::SVMDataPoint> PclSvmDataPointVector;

class PclScrewRecognitionTools
{
    public:
    PclScrewRecognitionTools(PclEyeParameters parameters) : m_parameters(parameters) {};

    PclScrewPtr localizeScrew(PCloudPtr pCloud);

    PclScrewPtr calculateResult(PCloudPtr pCloud);
    PCloudPtrVector euclideanClusterExtraction(PCloudPtr pCloud);
    void passThrough(PCloudPtr pCloud);
    void sacSegmentation(PCloudPtr pCloud);
    bool svmClassification(PCloudPtr pCloud);
    VFHFeaturePtr vfhEstimation(PCloudPtr pCloud);

    private:
    pcl::SVMData generateSvmDataOf(VFHFeaturePtr feature);

    PclEyeParameters& m_parameters;

    static const std::map<PclPassThroughFieldName, std::string> m_pt_field_name;
    static const std::map<PclSacModel, int> m_sac_model;
    static const std::map<PclSacMethod, int> m_sac_method;
};

#endif