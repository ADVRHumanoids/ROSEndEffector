#ifndef __ROSEE_UTILS_YAML__
#define __ROSEE_UTILS_YAML__

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>


namespace ROSEE { namespace Utils {

/**
 * @brief given a yaml node with a structure like
 *   - [1, 2, 3]
 *   - [4, 5, 6]
 *   "Convert" this structure into a eigen matrix
 * 
 * @todo is there a better way to do this?
 */
static Eigen::MatrixXd yamlMatrixToEigen(const YAML::Node &matrixNode) {
        
    //note: they say adding row by row to eigen is tremendously slow,
    // so we use vector
    std::vector<std::vector<float>> stdMat;
    stdMat = matrixNode.as<std::vector<std::vector<float>>>();

    Eigen::MatrixXd eigenMat(stdMat.size(), stdMat.at(0).size());
    
    for (int iRow = 0; iRow<stdMat.size(); iRow++) {
        for (int iCol = 0; iCol<stdMat.at(0).size(); iCol++) {
            eigenMat(iRow, iCol) =  stdMat.at(iRow).at(iCol);
        }
    }
    
    return eigenMat;
    
}

static Eigen::VectorXd yamlVectorToEigen(const YAML::Node &vectorNode) {
        
    //note: they say adding row by row to eigen is tremendously slow,
    // so we use vector
    std::vector<float> stdVect;
    stdVect = vectorNode.as<std::vector<float>>();

    Eigen::VectorXd eigenVec(stdVect.size());
    
    for (int i = 0; i<stdVect.size(); i++) {
        eigenVec(i) =  stdVect.at(i);
    }
    
    return eigenVec;
    
}

}

}

#endif // __ROSEE_UTILS_YAML__
