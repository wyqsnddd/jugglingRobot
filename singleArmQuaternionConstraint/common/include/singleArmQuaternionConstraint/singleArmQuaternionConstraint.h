# ifndef SINGLEARMQUATERNIONCONSTRAINT_H
# define SINGLEARMQUATERNIONCONSTRAINT_H

# include <gurobi_c++.h> 
# include <sstream>
# include <iostream>
# include <Eigen/Geometry>

# include <gurobiConstraint/gurobiConstraint.h>

class singleArmQuaternionConstraint : public gurobiConstraint{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private: 
  
  enum{
    DOF = 7,
    dim_ = 3
  };
  void initializeGurobi();  

  Eigen::Matrix<double, 3, 3>
    skew_build( const Eigen::Matrix<double, 3, 1>  &vector_in){
    //    Matrix_skew Matrix;
    Eigen::Matrix<double, 3, 3>  Matrix;
    
    Matrix << 0, -vector_in(2), vector_in(1),
      vector_in(2), 0, -vector_in(0),
      -vector_in(1), vector_in(0), 0;

    return Matrix;
  }

 public:
 singleArmQuaternionConstraint(
			       double &weight, 
			       const char* name, 
			       double &gain, 
			       double &margin,
			       double &bound,
			       std::vector< boost::shared_ptr<GRBVar> > &jointVVarPArray,
			       int sIndex,
			       boost::shared_ptr<GRBModel> &modelPointer, 
			       boost::shared_ptr<GRBQuadExpr> &objPointer, 
			       Eigen::Quaternion<double> currentQuaternion,
			       Eigen::Quaternion<double> desiredQuaternion,
			       const Eigen::Matrix<double, dim_, DOF> &quaternionGradient
			       ): gurobiConstraint( weight, 
						    name, 
						    false, //equality
						    gain, 
						    margin,
						    jointVVarPArray,
						    sIndex,
						    modelPointer, 
						    objPointer 
						    ){
    
    initializeGurobi();

    constraintUpdate(
		     currentQuaternion,
		     desiredQuaternion,		     
		     quaternionGradient );
  }
  
  ~singleArmQuaternionConstraint(){
  }
  void constraintUpdate(
			Eigen::Quaternion<double> currentQuaternion,
			Eigen::Quaternion<double> desiredQuaternion,
			const Eigen::Matrix<double, dim_, DOF> &gradient
			);
  
  void visualServoingConstraintUpdate(
				      Eigen::Quaternion<double> currentQuaternion,
				      Eigen::Quaternion<double> desiredQuaternion,
				      const Eigen::Matrix<double, dim_, DOF> &gradient
				      );

  void extractSlack(){
    for(int i = 0; i< dim_; i++)
      slackSolutions_[i] = slackPArray_[i]->get(GRB_DoubleAttr_X);
    /* std::cout<<"ee Height: slack is: "<<slackSolutions_[0]<<std::endl; */
  }
  std::vector<double> getSlackSolution(){
    return slackSolutions_;
  }
  
  double getObjectiveFunctional(){

    double temp(0); 
    
    for(int i = 0; i< dim_; i++)
      temp+=slackSolutions_[i]*slackSolutions_[i]*constraintWeight_;
    return temp;
  }


};

# endif
