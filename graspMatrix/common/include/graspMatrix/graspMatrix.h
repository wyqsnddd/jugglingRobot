# ifndef GRASPMATRIX_H
# define GRASPMATRIX_H

# include <Eigen/Geometry>
# include <adjointMap/adjointMap.h>

class graspMatrix{
  public:
 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  adjointMap * adgFCP_; 
  adjointMap * adgCVP_; 

 public: 
  graspMatrix(const Eigen::Matrix<double, 3, 3> &rotation, 
	      const Eigen::Matrix<double, 3, 1> &translation);
  graspMatrix();
  /* graspMatrix(const Eigen::Affine3d &transform); */
  ~graspMatrix(){
    free(adgFCP_);
    free(adgCVP_);
  }
  // updater
  void updateFC(const Eigen::Matrix<double, 3, 3>  &rotation){
    adgFCP_->updateRotation(rotation);
  }
  void updateCV(const Eigen::Matrix<double, 3, 1>  &translation){
    adgCVP_->updateTranslation(translation);
  }
  void updateFV(const Eigen::Affine3d &transform){
    adgFCP_->updateRotation(transform.rotation());
    adgCVP_->updateTranslation(transform.translation());
  }
  
  // reader
  Eigen::Matrix<double, 6, 6>  readFV(){
    // Notice the use of the left multiplication, which applies for homogeneous transform as well
    return adgCVP_->read()*adgFCP_->read();
  }
  Eigen::Matrix<double, 6, 6>  readFC(){
    // Notice the use of the left multiplication, which applies for homogeneous transform as well
    return adgFCP_->read();
  }
  Eigen::Matrix<double, 6, 6>  readCV(){
    // Notice the use of the left multiplication, which applies for homogeneous transform as well
    return adgCVP_->read();
  }
  // for the inverse, we use the right multiplication 
  Eigen::Matrix<double, 6, 6>  readVF(){
    return adgFCP_->inverse()*adgCVP_->inverse();
  }
  Eigen::Matrix<double, 6, 6>  readVC(){
    // Notice the use of the left multiplication, which applies for homogeneous transform as well
    return adgCVP_->inverse();
  }
  Eigen::Matrix<double, 6, 6>  readCF(){
    // Notice the use of the left multiplication, which applies for homogeneous transform as well
    return adgFCP_->inverse();
  }
  
};
# endif
