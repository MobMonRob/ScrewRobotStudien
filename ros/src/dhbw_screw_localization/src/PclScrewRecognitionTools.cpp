#include "dhbw_screw_localization/PclScrewRecognitionTools.h"

const std::map<PclPassThroughFieldName, std::string> PclScrewRecognitionTools::m_pt_field_name = { {PclPassThroughFieldName::X, "x"}, {PclPassThroughFieldName::Y, "y"}, {PclPassThroughFieldName::Z, "z"} };
const std::map<PclSacModel, int> PclScrewRecognitionTools::m_sac_model = { {PclSacModel::PLANE, pcl::SACMODEL_PLANE} };
const std::map<PclSacMethod, int> PclScrewRecognitionTools::m_sac_method = { {PclSacMethod::RANSAC, pcl::SAC_RANSAC} };

PclScrewPtr PclScrewRecognitionTools::localizeScrew(PCloudPtr pCloud)
{
    if(pCloud->points.size() == 0) return PclScrewPtr(nullptr);
    passThrough(pCloud);

    if(pCloud->points.size() == 0) return PclScrewPtr(nullptr);
    sacSegmentation(pCloud);
    
    if(pCloud->points.size() == 0) return PclScrewPtr(nullptr);
    for(PCloudPtr cluster : euclideanClusterExtraction(pCloud))
    {
        if(svmClassification(cluster))
        {
            return calculateResult(cluster);
        }
    }
    return PclScrewPtr(nullptr);
}

void PclScrewRecognitionTools::passThrough(PCloudPtr pCloud)
{
    for(PclPassThroughParameters parameters : m_parameters.pt)
    {
        pcl::PassThrough<pcl::PointXYZ> passthrough;
        
        PCloudPtr filteredPCloud(new PCloud());

        passthrough.setFilterFieldName(m_pt_field_name.at(parameters.filterFieldName));
        passthrough.setFilterLimits(parameters.filterLimitMin, parameters.filterLimitMax);
        passthrough.setFilterLimitsNegative(false);
        passthrough.setInputCloud(pCloud);
        passthrough.filter(*filteredPCloud);
        copyPointCloud(*filteredPCloud, *pCloud);
    }

    return;
}

void PclScrewRecognitionTools::sacSegmentation(PCloudPtr pCloud)
{
    for(PclSacSegmentationParameters parameters : m_parameters.sacs)
    {
        pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation;
        pcl::ExtractIndices<pcl::PointXYZ> extract_indices;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        PCloudPtr filteredPCloud(new PCloud());

        sac_segmentation.setOptimizeCoefficients(true);
        sac_segmentation.setModelType(m_sac_model.at(parameters.model));
        sac_segmentation.setMethodType(m_sac_method.at(parameters.method));
        sac_segmentation.setDistanceThreshold(parameters.distanceThreshold);
        sac_segmentation.setInputCloud(pCloud);
        sac_segmentation.segment(*inliers, *coefficients);

        extract_indices.setIndices(inliers);
        extract_indices.setNegative(true);
        extract_indices.setInputCloud(pCloud);
        extract_indices.filter(*filteredPCloud);
        copyPointCloud(*filteredPCloud, *pCloud);
    }

    return;
}

PCloudPtrVector PclScrewRecognitionTools::euclideanClusterExtraction(PCloudPtr pCloud)
{
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extraction;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud(pCloud);
    
    std::vector<pcl::PointIndices> clusters;
    PCloudPtrVector pCloudClusters;

    euclidean_cluster_extraction.setClusterTolerance(m_parameters.ece.tolerance);
    euclidean_cluster_extraction.setMinClusterSize(m_parameters.ece.minSize);
    euclidean_cluster_extraction.setMaxClusterSize(m_parameters.ece.maxSize);
    euclidean_cluster_extraction.setSearchMethod(kdTree);
    euclidean_cluster_extraction.setInputCloud(pCloud);
    euclidean_cluster_extraction.extract(clusters);

    for(pcl::PointIndices clusterInliers: clusters)
    {
        PCloudPtr pCloudCluster(new PCloud);
        for(int pointIndex: clusterInliers.indices)
        {
            pCloudCluster->points.push_back(pCloud->points.at(pointIndex));
        }
        pCloudCluster->width = pCloudCluster->points.size();
        pCloudCluster->height = 1;
        pCloudCluster->is_dense = true;
        pCloudClusters.push_back(pCloudCluster);
    }

    return pCloudClusters;
}

VFHFeaturePtr PclScrewRecognitionTools::vfhEstimation(PCloudPtr pCloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh_estimation;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTreeNormals(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTreeFeatures(new pcl::search::KdTree<pcl::PointXYZ>());
    VFHFeaturePtr features(new VFHFeature);
    
    normal_estimation.setSearchMethod(kdTreeNormals);
    normal_estimation.setRadiusSearch(m_parameters.vfhe.normalsRadiusSearch);
    normal_estimation.setInputCloud(pCloud);
    normal_estimation.compute(*normals);

    vfh_estimation.setSearchMethod(kdTreeFeatures);
    vfh_estimation.setInputNormals(normals);
    vfh_estimation.setInputCloud(pCloud);
    vfh_estimation.compute(*features);
    
    return features;
}

bool PclScrewRecognitionTools::svmClassification(PCloudPtr pCloud)
{   
    if(m_parameters.svmc.training) return true;
    pcl::SVMClassify svm_classifier;
    svm_classifier.loadClassifierModel(m_parameters.svmc.pathToModel.c_str());

    VFHFeaturePtr feature = vfhEstimation(pCloud);
    PclSvmData svmData = generateSvmDataOf(feature);
    bool result = svm_classifier.classification(svmData)[0] == m_parameters.svmc.positive;

    return result;
}

PclSvmData PclScrewRecognitionTools::generateSvmDataOf(VFHFeaturePtr feature)
{
    PclSvmData svmData;
    PclSvmDataPointVector svmDataPoints;

    for(int i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
    {
        pcl::SVMDataPoint svmDataPoint;
        svmDataPoint.idx = i;
        svmDataPoint.value = feature->points[0].histogram[i];
        svmDataPoints.push_back(svmDataPoint);
    }
    svmData.label = 1.0;
    svmData.SV = svmDataPoints;

    return svmData;
}

PclScrewPtr PclScrewRecognitionTools::calculateResult(PCloudPtr pCloud)
{
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    Eigen::Matrix4f identityMatrix(Eigen::Matrix4f::Identity());
    pcl::PointCloud<pcl::PointXYZ> transformedPCloud;
    pcl::PointXYZ minPoint;
    pcl::PointXYZ maxPoint;
    
    pcl::compute3DCentroid(*pCloud, centroid);
    pcl::computeCovarianceMatrixNormalized(*pCloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    identityMatrix.block<3,3>(0,0) = eigenVectors.transpose();
    identityMatrix.block<3,1>(0,3) = -1.0f * (identityMatrix.block<3,3>(0,0) * centroid.head<3>());
    
    pcl::transformPointCloud(*pCloud, transformedPCloud, identityMatrix);
    pcl::getMinMax3D(transformedPCloud, minPoint, maxPoint);
    
    const Eigen::Vector3f meanDiagonale = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    const Eigen::Quaternionf quaternion(eigenVectors);
    const Eigen::Vector3f translation = eigenVectors * meanDiagonale + centroid.head<3>();

    PclScrewPtr screw = std::make_shared<PclScrew>(*pCloud, minPoint, maxPoint, quaternion, translation);
    
    return screw; 
}
