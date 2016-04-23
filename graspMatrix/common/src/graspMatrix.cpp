# include <graspMatrix/graspMatrix.h>

graspMatrix::graspMatrix(const Eigen::Matrix<double, 3, 3> &rotation, 
			 const Eigen::Matrix<double, 3, 1> &translation){
  
  adgFCP_ = new adjointMap(rotation); 
  adgCVP_ = new adjointMap(translation); 
}

graspMatrix::graspMatrix(){
  
  adgFCP_ = new adjointMap(); 
  adgCVP_ = new adjointMap(); 
}
// graspMatrix::graspMatrix(const Eigen::Affine3d &transform){
  
//   adgFCP_ = new adjointMap(transform.rotation()); 
//   adgCOP_ = new adjointMap(transform.translation());
// }
