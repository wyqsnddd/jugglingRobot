# include <singleArmTranslationConstraint/singleArmTranslationConstraint.h>

void singleArmTranslationConstraint::initializeGurobi(){
  slackSolutions_.resize(dim_);
  slackPArray_.resize(dim_);

  for(unsigned int i = 0; i < dim_; i++){
    slackPArray_[i].reset(new GRBVar);
    std::ostringstream temp;
    temp<< name_<< "  slack variable "<<i;
    //	      " gripper distance constraint slack variable: "<<i;
    *slackPArray_[i] = modelPointer_->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, temp.str().c_str() );
  }

  // Add to the objective
  for(unsigned int i =0; i<dim_; i++ ){
    *objPointer_ += constraintWeight_
      *(*slackPArray_[i])
      *(*slackPArray_[i]);
  }

  modelPointer_->update();
  // Initialize the constraint
  constrPArray_.resize(dim_);
  for(unsigned int i = 0; i<dim_; i++ ){
    GRBLinExpr test = 0;

    // Include two arms
    for( int j = 0; j<DOF; j++){
      int k = j + sIndex_;
      //      test += *jointVVarPArray[j]*0.0;
      test += *(*jointVVarPArrayPointer_)[k]*0.0;
    }
	    
    test += *slackPArray_[i];

    std::ostringstream tempCon;
    tempCon<< name_<<i;
    
    constrPArray_[i].reset(new GRBConstr);
    *constrPArray_[i] = modelPointer_->addConstr(test,
						 GRB_EQUAL,
						 composeBound(0.0),// place holders
						 tempCon.str().c_str());

  }// loop over one constraint

  modelPointer_->update();


}  
void singleArmTranslationConstraint:: constraintUpdate(const Eigen::Matrix<double, 3, 1> &currentTranslation,
						       const Eigen::Matrix<double, 3, 1> &desiredTranslation,
						       const Eigen::Matrix<double, 3, 1> &timeDerivative,
						       const Eigen::Matrix<double, 6, DOF> &gradient
						       ){

  // update function measure
  Eigen::Matrix<double, 3, 1> measure;
  measure.setZero();
  measure = currentTranslation - desiredTranslation;
  


  measureNorm_ = measure.norm();

  // update the constraint gradient
  Eigen::Matrix<double, 3, DOF> tempGradient;
  tempGradient.setZero();
  tempGradient = gradient.block<3,DOF>(0,0);
    
  // set up the gradient
  for(int j = 0; j<dim_; j++){
    for(int i = 0; i<DOF; i++){
      // int k = i + sIndex_;
      modelPointer_->chgCoeff(
			      *constrPArray_[j],
			      *(*jointVVarPArrayPointer_)[i],
			      tempGradient(j, i)
			      // gradient(j, i)
			      );	
    }// finish one constraint
  }// finish all constraints


  Eigen::Matrix<double, 3, 1> rhs;
  rhs.setZero();
  // update the rhs
  for(int j = 0; j<dim_; j++){
    
    rhs(j) =  - gain_*( currentTranslation(j) - desiredTranslation(j) ); 

    constrPArray_[j]->set(GRB_DoubleAttr_RHS, 
			  rhs(j)
			  );

  }

  // std::cout<<"error   X: "<<measure(0)           <<" error   Y: "<<measure(1)           <<" error   Z: "<<measure(2)<<std::endl;
  // std::cout<<"error   norm: "<<measureNorm_<<std::endl;
  // std::cout<<"seperation line:---------------------------- "<<std::endl;

}
