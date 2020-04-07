#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
#include <pcl/ml/svm_wrapper.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <dirent.h>
#include <iostream>
#include <string>
#include <vector>

typedef std::queue<std::string> QueueOfStrings;
typedef pcl::PointCloud<pcl::PointXYZ> PCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCloudPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> FPFHFeature;
typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFHFeaturePtr;
typedef pcl::PointCloud<pcl::VFHSignature308> VFHFeature;
typedef pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHFeaturePtr;
typedef std::vector<pcl::SVMData> SvmDataVector;
typedef pcl::SVMDataPoint SvmDataPoint;
typedef std::vector<pcl::SVMDataPoint> SvmDataPointVector;

QueueOfStrings getFileNamesInDictionary(const char* path);
FPFHFeaturePtr calculateFPFHFeature(PCloudPtr pCloud);
VFHFeaturePtr calculateVFHFeature(PCloudPtr pCloud);
pcl::SVMData generateSVMDataOf(VFHFeaturePtr feature);

int main(int argc, char* argv[])
{
    std::string input_dir = argv[1];
    std::string output_file = argv[2];

    QueueOfStrings ply_paths = getFileNamesInDictionary(input_dir.c_str());

    PCloudPtr pCloud(new PCloud);
    SvmDataVector training_set;

    while(ply_paths.size() > 0 && pcl::io::loadPLYFile<pcl::PointXYZ>(ply_paths.front(), *pCloud) != -1)
    {
        VFHFeaturePtr feature = calculateVFHFeature(pCloud);

        training_set.push_back(generateSVMDataOf(feature));
        ply_paths.pop();
    }

    pcl::SVMTrain svm_trainer;
    pcl::SVMParam svm_parameters;
    
    svm_parameters.svm_type = ONE_CLASS;
    svm_parameters.kernel_type = POLY;
    svm_trainer.setParameters(svm_parameters);
    svm_trainer.setInputTrainingSet(training_set);

    if(svm_trainer.trainClassifier())
    {
        std::cout << "Successfully tranined. \n";
        svm_trainer.saveClassifierModel(output_file.c_str());
        std::cout << "Model saved to: " << output_file << "\n";
    }
    else
    {
        std::cout << "Not trained. \n";
        return 0;
    }
}

QueueOfStrings getFileNamesInDictionary(const char* path)
{
    std::queue<std::string> ply_paths;
    DIR* dirp = opendir(path);
    struct dirent * dp;
    std::cout << "Readed files: \n";
    while ((dp = readdir(dirp)) != NULL) 
    {
        std::string filename = dp->d_name;
        std::string path_filename = path + filename;
        ply_paths.push(path_filename);
        std::cout << "\t" << path_filename << "\n";
    }
    ply_paths.pop();
    ply_paths.pop();
    closedir(dirp);
    return ply_paths;
}

VFHFeaturePtr calculateVFHFeature(PCloudPtr pCloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(0.05);
    normal_estimation.setInputCloud(pCloud);
    normal_estimation.compute(*normals);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());
    VFHFeaturePtr features(new VFHFeature);
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(pCloud);
    vfh.setInputNormals(normals);
    vfh.setSearchMethod(kdTree);
    vfh.compute(*features);
    return features;
}

pcl::SVMData generateSVMDataOf(VFHFeaturePtr feature)
{
    pcl::SVMData svmData;
    SvmDataPointVector svmDataPoints;

    for(int i = 0; i < pcl::VFHSignature308::descriptorSize(); i++)
    {
        SvmDataPoint svmDataPoint;
        svmDataPoint.idx = i;
        svmDataPoint.value = feature->points[0].histogram[i];
        svmDataPoints.push_back(svmDataPoint);
    }
    svmData.label = 1.0;
    svmData.SV = svmDataPoints;
    return svmData;
}
