# ifndef SINGLEARMTRANSLATIONCONSTRAINT_H
# define SINGLEARMTRANSLATIONCONSTRAINT_H
# include <gurobi_c++.h> 
# include <sstream>
# include <iostream>
# include <Eigen/Geometry>

# include <gurobiConstraint/gurobiConstraint.h>
class singleArmTranslationConstraint : public gurobiConstraint{

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
 private: 
  
  enum{
    DOF = 7,
    dim_ = 3
  };

  void initializeGurobi();  
  
  double measureNorm_;
 public:

 singleArmTranslationConstraint(
				double &weight, 
				const char* name, 
				double &gain, 
				double &margin,
				std::vector< boost::shared_ptr<GRBVar> > &jointVVarPArray,
				int sIndex,
				boost::shared_ptr<GRBModel> &modelPointer, 
				boost::shared_ptr<GRBQuadExpr> &objPointer, 
				const Eigen::Matrix<double, 3, 1> &currentTranslation,
				const Eigen::Matrix<double, 3, 1> &desiredTranslation,
				const Eigen::Matrix<double, 3, 1> &timeDerivative,
				const Eigen::Matrix<double, 6, DOF> &gradient
				): gurobiConstraint( weight, 
						     name, 
						     false, //equality
						     gain, 
						     /* 0.0, */
						     margin,
						     jointVVarPArray,
						     sIndex,
						     modelPointer, 
						     objPointer 
						     ){
    measureNorm_ = 0.0;
    
    initializeGurobi();

    constraintUpdate(
		     currentTranslation,
		     desiredTranslation,
		     timeDerivative,
		     gradient);

  } // end of constructor 
  ~singleArmTranslationConstraint(){
  }
  void constraintUpdate(const Eigen::Matrix<double, 3, 1> &currentTranslation,
			const Eigen::Matrix<double, 3, 1> &desiredTranslation,
			const Eigen::Matrix<double, 3, 1> &timeDerivative,
			const Eigen::Matrix<double, 6, DOF> &gradient
			);
  double readMeasureNorm(){
    return measureNorm_;
  }
  void extractSlack(){
    for(int i = 0; i<dim_; i++)
      slackSolutions_[i] = slackPArray_[i]->get(GRB_DoubleAttr_X);
    /* std::cout<<"ee Height: slack is: "<<slackSolutions_[0]<<std::endl; */
  }
  double getObjectiveFunctional(){

    double objective = 0.0;
    for(int i = 0; i<dim_; i++){
      objective += slackSolutions_[i]*slackSolutions_[i]*constraintWeight_;
    }
      
    return objective;
      
  }
  
  
};
# endif
