#ifndef PCLEYEPARAMETERS_H_
#define PCLEYEPARAMETERS_H_

#include <string>
#include <vector>

struct PclEuclideanClusterExtractionParameters
{
    float tolerance;
    int minSize;
    int maxSize;
};

enum PclPassThroughFieldName : unsigned int
{ 
    X = 0,
    Y = 1,
    Z = 2
};

struct PclPassThroughParameters
{
    PclPassThroughFieldName filterFieldName;
    float filterLimitMin;
    float filterLimitMax;
};

enum PclSacModel : unsigned int
{ 
    PLANE 
};

enum PclSacMethod : unsigned int
{ 
    RANSAC 
};

struct PclSacSegmentationParameters
{
    PclSacModel model;
    PclSacMethod method;
    float distanceThreshold;
};

struct PclSvmClassificationParameters
{
    bool training;
    std::string pathToModel;
    double positive;
};

struct PclVfhEstimationParameters
{
    float normalsRadiusSearch;
};

struct PclEyeParameters
{
    PclEuclideanClusterExtractionParameters ece;
    std::vector<PclPassThroughParameters> pt;
    std::vector<PclSacSegmentationParameters> sacs;
    PclSvmClassificationParameters svmc;
    PclVfhEstimationParameters vfhe;
};

#endif