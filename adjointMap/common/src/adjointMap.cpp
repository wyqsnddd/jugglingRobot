# include <adjointMap/adjointMap.h>
adjointMap::adjointMap(const Eigen::Matrix<double, 6, 6> inputAdjointMap){
  adjointTransform_.setZero();

  adjointTransform_.setIdentity();

  Eigen::Matrix<double, 3, 3> skew_matrix = 
     inputAdjointMap.block<3,3>(0,3)
      *inputAdjointMap.block<3,3>(0,0).transpose();
  Eigen::Matrix<double, 3, 1> inputTranslation;
  inputTranslation =  unskew(skew_matrix );

  
  adjointTransform_.block<3,3>(0,0) << inputAdjointMap.block<3,3>(0,0);
  
  adjointTransform_.block<3,3>(3,3) << inputAdjointMap.block<3,3>(0,0);

  adjointTransform_.block<3,3>(0,3) << 
    skew_build(inputTranslation)*inputAdjointMap.block<3,3>(0,0);

}


adjointMap::adjointMap(){
  adjointTransform_.setZero();
  adjointTransform_.setIdentity();
  
}

adjointMap::adjointMap(const Eigen::Affine3d &transform){
  adjointTransform_.setZero();
  adjointTransform_.setIdentity();
  
  adjointTransform_.block<3,3>(0,0) << transform.rotation();
  
  adjointTransform_.block<3,3>(3,3) << transform.rotation();

  adjointTransform_.block<3,3>(0,3) << 
    skew_build(transform.translation())*transform.rotation();
  
}
adjointMap::adjointMap(const Eigen::Matrix<double, 3, 3> &rotation, const Eigen::Matrix<double, 3, 1> &translation){

  adjointTransform_.setZero();
  adjointTransform_.setIdentity();
  
  adjointTransform_.block<3,3>(0,0) << rotation;
  
  adjointTransform_.block<3,3>(3,3) << rotation;

  adjointTransform_.block<3,3>(0,3) << skew_build(translation)*rotation;
  
}
adjointMap::adjointMap(const Eigen::Matrix<double, 3, 3> &rotation){

  adjointTransform_.setZero();
  adjointTransform_.setIdentity();
  
  adjointTransform_.block<3,3>(0,0) << rotation;
  
  adjointTransform_.block<3,3>(3,3) << rotation;

  
}
adjointMap::adjointMap(const Eigen::Matrix<double, 3, 1> &translation){

  adjointTransform_.setZero();
  adjointTransform_.setIdentity();
  adjointTransform_.block<3,3>(0,3) << skew_build(translation);
  
}
Eigen::Matrix<double, 6, 6>  adjointMap::inverse(){
  
  Eigen::Matrix<double, 6, 6> sample;
  sample.setIdentity();
  sample.block<3,3>(0,0)<<readRotation().transpose();
  sample.block<3,3>(3,3)<<readRotation().transpose();

  sample.block<3,3>(0,3)<<-readRotation().transpose() 
    *adjointTransform_.block<3,3>(0,3)
    *readRotation().transpose();
  return sample;
}
Eigen::Matrix<double, 3, 1> 
adjointMap::unskew( const Eigen::Matrix<double, 3, 3>  &skewMatrix){

    Eigen::Matrix<double, 3, 1>  originalVector;
    
    originalVector(0) = 0.5*(-skewMatrix(1,2) + skewMatrix(2,1));
    originalVector(1) = 0.5*(skewMatrix(0,2) - skewMatrix(2,0));
    originalVector(2) = 0.5*(-skewMatrix(0,1) + skewMatrix(1,0));
    
    return originalVector;

}

Eigen::Matrix<double, 3, 3> 
adjointMap::skew_build( const Eigen::Matrix<double, 3, 1>  &vector_in){
    //    Matrix_skew Matrix;
    Eigen::Matrix<double, 3, 3>  Matrix;
    
    Matrix << 0, -vector_in(2), vector_in(1),
      vector_in(2), 0, -vector_in(0),
      -vector_in(1), vector_in(0), 0;

    return Matrix;
  } 
void adjointMap::updateRotation(const Eigen::Matrix<double, 3, 3> &rotation){

  adjointTransform_.block<3,3>(0,3) <<skew_build(readTranslation())*rotation;
  adjointTransform_.block<3,3>(0,0) << rotation;
  adjointTransform_.block<3,3>(3,3) << rotation;
}

void adjointMap::updateTranslation(const Eigen::Matrix<double, 3, 1> &translation){
  adjointTransform_.block<3,3>(0,3) <<skew_build(translation)*readRotation();
}
void adjointMap::updateTransform(const Eigen::Affine3d &transform){
  adjointTransform_.block<3,3>(0,0) << transform.rotation();
  
  adjointTransform_.block<3,3>(3,3) << transform.rotation();

  adjointTransform_.block<3,3>(0,3) << skew_build(transform.translation())*transform.rotation();

}

void adjointMap::updateTransform(const Eigen::Matrix<double, 3, 3> &rotation,
				 const Eigen::Matrix<double, 3, 1> &translation){

  adjointTransform_.block<3,3>(0,0) << rotation;
  
  adjointTransform_.block<3,3>(3,3) << rotation;

  adjointTransform_.block<3,3>(0,3) << skew_build(translation)*rotation;


}

