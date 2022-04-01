#ifndef __ROSEE_UTILS_EIGEN__
#define __ROSEE_UTILS_EIGEN__

#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>

namespace ROSEE
{

namespace Utils
{

/**
 * @brief Utility to fill the Float32MultiArray ROS message from an eigen matrix.
 * The Float32MultiArray matrix will be stored in column major, independently from the
 * eigen matrix that can be row major (even by default eigen matrix is column major,
 * it can be templatizate specifing the row major)
 */
static std_msgs::Float32MultiArray eigenMatrixToFloat32MultiArray (Eigen::MatrixXd eigenMatrix) {
    
    std_msgs::Float32MultiArray rosMatrix;
    
    unsigned int nRow = eigenMatrix.rows();
    unsigned int nCol = eigenMatrix.cols();
    unsigned int size = nRow*nCol;
    
    if (size == 0){ //no reason to continue
        std::cerr << "[Utils::eigenMatrixToFloat32MultiArray] eigenMatrix passed has size 0" << std::endl;
        return rosMatrix;
    }
    
    rosMatrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rosMatrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
    rosMatrix.layout.dim[0].label = "column";
    rosMatrix.layout.dim[0].size = nCol;
    rosMatrix.layout.dim[0].stride = nCol*nRow;
    
    rosMatrix.layout.dim[1].label = "row";
    rosMatrix.layout.dim[1].size = nRow;
    rosMatrix.layout.dim[1].stride = nRow;
    
    rosMatrix.layout.data_offset = 0;
  
    rosMatrix.data.resize(size);
    int posRow = 0;
    for (int iCol=0; iCol<nCol; iCol++){
        for (int iRow=0; iRow<nRow; iRow++){
            posRow = iCol*nRow;
            rosMatrix.data.at(posRow + iRow) = eigenMatrix(iRow,iCol);
        }
    }
    
    return rosMatrix;

    
}

static std::vector<float> eigenVectorToStdVector (Eigen::VectorXd eigenVector) {
    
    std::vector<float> stdVect(eigenVector.data(), eigenVector.data() + eigenVector.size());
    
    return stdVect;
}

}} //namespaces

#endif // __ROSEE_UTILS_EIGEN__
