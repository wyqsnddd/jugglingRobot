# include <singleArmQuaternionConstraint/singleArmQuaternionConstraint.h>

void singleArmQuaternionConstraint::initializeGurobi(){
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




} // end of gurobiIni();

void singleArmQuaternionConstraint::visualServoingConstraintUpdate(
								   Eigen::Quaternion<double> currentQuaternion,
								   Eigen::Quaternion<double> desiredQuaternion,
								   const Eigen::Matrix<double, dim_, DOF> &gradient
								   ){

  Eigen::Quaternion<double> errorQuaternion;
  errorQuaternion = currentQuaternion.inverse()*desiredQuaternion;

  // Eigen::Matrix<double, 3, 1> error = -gain_ *errorQuaternion.vec();
  Eigen::Matrix<double, 3, 1> error = -gain_ *(currentQuaternion.vec() - desiredQuaternion.vec());


  // std::cout<<"The  quaternion error is:"<<std::endl<<error<<std::endl;
  // We may add a rate term: 
  // error  = error - rate
  
  for(int i = 0; i<dim_; i++){
    constrPArray_[i]->set(GRB_DoubleAttr_RHS, 
			  error(i)
			  );
  }

  Eigen::Matrix<double, dim_, DOF> tempGradient;
  Eigen::Matrix<double, 3, 3> identityMatrix;
    
  identityMatrix.setIdentity();

  
  tempGradient = 0.5*(
		      currentQuaternion.w()*identityMatrix 
		      - skew_build(currentQuaternion.vec())
		      )*gradient;
  
  // set up the gradient
  for(int i = 0; i<dim_; i++){
    for(int j = 0; j<DOF; j++){
      modelPointer_->chgCoeff(
			      *constrPArray_[i],
			      *(*jointVVarPArrayPointer_)[j],
			      tempGradient(i,j)
			      );	
    }
  }


  
}
void singleArmQuaternionConstraint::constraintUpdate(
						     Eigen::Quaternion<double> currentQuaternion,
						     Eigen::Quaternion<double> desiredQuaternion,
						     const Eigen::Matrix<double, dim_, DOF> &gradient
						     ){

  // calculate the orientation error
  
  // Eigen::Matrix<double, 3, 1> error = -gain_ *(
  // 					       currentQuaternion.w()*desiredQuaternion.vec()
  // 					       - desiredQuaternion.w()*currentQuaternion.vec() 
  // 					       -skew_build(currentQuaternion.vec())*desiredQuaternion.vec()
  // 					       ); 
  // Eigen::Matrix<double, 3, 1> error = -gain_ *(
  // 					       currentQuaternion.w()*desiredQuaternion.vec()
  // 					       - desiredQuaternion.w()*currentQuaternion.vec() 
  // 					       -skew_build(currentQuaternion.vec())*desiredQuaternion.vec()
  // 					       ); 
  Eigen::Matrix<double, 3, 1> error = -gain_ *(
					       (currentQuaternion.inverse()*desiredQuaternion).vec()
					       // currentQuaternion.w()*desiredQuaternion.vec()
					       // - desiredQuaternion.w()*currentQuaternion.vec() 
					       // -skew_build(currentQuaternion.vec())*desiredQuaternion.vec()
					       ); 
  // std::cout<<"The  quaternion error is:"<<std::endl<<error<<std::endl;
  
  for(int i = 0; i<dim_; i++){
    constrPArray_[i]->set(GRB_DoubleAttr_RHS, 
			  error(i)
			  );
  }

  Eigen::Matrix<double, dim_, DOF> tempGradient;
  

  // plan 1 direct geometric jacobian
  // tempGradient = gradient;

  // plan 2 modified jacobian 

  Eigen::Matrix<double, 3, 3> identityMatrix;
  identityMatrix.setIdentity();
  tempGradient = (
  		  -0.5*desiredQuaternion.vec()*currentQuaternion.vec().transpose()
  		  -0.5*desiredQuaternion.w()*(
  					      currentQuaternion.w()*identityMatrix - skew_build(currentQuaternion.vec())
  					      )
  		  + 0.5*skew_build(desiredQuaternion.vec())*(
  							   currentQuaternion.w()*identityMatrix - skew_build(currentQuaternion.vec())
  							   )
  		  )*gradient;

  // set up the gradient
  for(int i = 0; i<dim_; i++){
    for(int j = 0; j<DOF; j++){
      modelPointer_->chgCoeff(
			      *constrPArray_[i],
			      *(*jointVVarPArrayPointer_)[j],
			      tempGradient(i,j)
			      );	
    }
  }
}

