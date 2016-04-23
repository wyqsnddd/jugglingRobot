# ifndef ADJOINTMAP_H
# define ADJOINTMAP_H

# include <Eigen/Geometry>
/* # include <boost/shared_ptr.hpp> */

class adjointMap{
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  
  Eigen::Matrix<double, 6, 6> adjointTransform_;

  // a vector to a skew-symmetric matrix 
  Eigen::Matrix<double, 3, 3> 
    skew_build( const Eigen::Matrix<double, 3, 1>  &vector_in);
  Eigen::Matrix<double, 3, 1> 
    unskew( const Eigen::Matrix<double, 3, 3>  &skewMatrix);
  
 public:
  
  adjointMap();
  
  adjointMap(const Eigen::Matrix<double, 6, 6> inputAdjointMap);
  
  adjointMap(const Eigen::Affine3d &transform);
  adjointMap(const Eigen::Matrix<double, 3, 3> &rotation, const Eigen::Matrix<double, 3, 1> &translation);
  adjointMap(const Eigen::Matrix<double, 3, 3> &rotation);
  adjointMap(const Eigen::Matrix<double, 3, 1> &translation);
  ~adjointMap(){
  }
  // Public update API
  void updateRotation(const Eigen::Matrix<double, 3, 3> &rotation);
  void updateTranslation(const Eigen::Matrix<double, 3, 1> &translation);
  void updateTransform(const Eigen::Affine3d &transform);
  void updateTransform(const Eigen::Matrix<double, 3, 3> &rotation,
		       const Eigen::Matrix<double, 3, 1> &translation);
  
  // Public reader API
  Eigen::Matrix<double, 6, 6>  inverse();

  Eigen::Matrix<double, 6, 6>  read(){
    return adjointTransform_;
  }
  Eigen::Matrix<double, 3, 3>  readRotation(){
    return adjointTransform_.block<3,3>(0,0);
  }
  Eigen::Matrix<double, 3, 1>  readTranslation(){
    
    Eigen::Matrix<double, 3, 3> skew_matrix = 
     adjointTransform_.block<3,3>(0,3)
      *adjointTransform_.block<3,3>(0,0).transpose();
    return unskew(skew_matrix );
  }
};
# endif
