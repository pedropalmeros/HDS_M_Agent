//  created:    2011/05/01
//  filename:   Law.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/



#include "Controller4_1Class.h"
#include <cvmatrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <GroupBox.h>
#include <AhrsData.h>
#include <DataPlot1D.h>
#include <math.h>
#include <ImuData.h>
#include <iostream>
#include <cmath>
#include <TabWidget.h>
#include <Tab.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;
using namespace std;

namespace flair { namespace filter {


Controller4_1Class::Controller4_1Class(TabWidget* tabwidget,string name) :  ControlLaw(tabwidget,name,3) {
	
	
	Tab *PositionTab = new Tab(tabwidget,"Position");
	TabWidget *LateralTabs = new TabWidget(PositionTab->NewRow(),"laws1");
	PositionsSingle = new Tab(LateralTabs,"Positions");
	GainsSingle = new Tab(LateralTabs,"Gains"); 
	GraphesPositionTab = new Tab(LateralTabs,"Graphs");
  
    input=new cvmatrix(this,33,1,floatType,name);
    
		cvmatrix_descriptor* desc=new cvmatrix_descriptor(33,1);
        desc->SetElementName(0,0,"kp_roll");
        desc->SetElementName(1,0,"kp_pitch");
        desc->SetElementName(2,0,"kp_yaw");
        desc->SetElementName(3,0,"kd_roll");
        desc->SetElementName(4,0,"kd_pitch");
        desc->SetElementName(5,0,"kd_yaw");

        desc->SetElementName(6,0,"roll");
        desc->SetElementName(7,0,"refroll");

        desc->SetElementName(8,0,"pitch");
        desc->SetElementName(9,0,"refpitch");

        desc->SetElementName(10,0,"yaw");
        desc->SetElementName(11,0,"refyaw");

        desc->SetElementName(12,0,"droll");
        desc->SetElementName(13,0,"refdroll");

        desc->SetElementName(14,0,"dpitch");
        desc->SetElementName(15,0,"refdpitch");

        desc->SetElementName(16,0,"dyaw");
        desc->SetElementName(17,0,"refdyaw");

        desc->SetElementName(18,0,"Tau_x");
        desc->SetElementName(19,0,"Tau_y");
        desc->SetElementName(20,0,"Tau_z");

        desc->SetElementName(21,0,"a1");
        desc->SetElementName(22,0,"a2");
        desc->SetElementName(23,0,"a3");
        desc->SetElementName(24,0,"a4");
        desc->SetElementName(25,0,"a5");
        desc->SetElementName(26,0,"");
        desc->SetElementName(27,0,"a1");
        desc->SetElementName(28,0,"a2");
        desc->SetElementName(29,0,"a3");
        desc->SetElementName(30,0,"a4");
        desc->SetElementName(31,0,"a5");
        desc->SetElementName(32,0,"");
        
        
    	state=new cvmatrix(this,desc,floatType,name);

        AddDataToLog(state);

    Reset();
 
     // Regu Gain Values Spinboxes
    GroupBox* HomePositionGroupBox = new GroupBox(PositionsSingle -> NewRow(),"Home Positions");
		homePosx = new DoubleSpinBox(HomePositionGroupBox -> NewRow(),"Pos x: ",-5,5,0.01,3);
		homePosy = new DoubleSpinBox(HomePositionGroupBox -> NewRow(),"Pos y: ",-5,5,0.01,3);
    
    GroupBox* UserPositionGroupBox = new GroupBox(PositionsSingle -> NewRow(),"User Positions");
		userPosx = new DoubleSpinBox(UserPositionGroupBox -> NewRow(),"Pos x: ",-5,5,0.01,3);
		userPosy = new DoubleSpinBox(UserPositionGroupBox -> NewRow(),"Pos y: ",-5,5,0.01,3);
		    
    
    GroupBox* AttitudeGainsGroupBox = new GroupBox(GainsSingle -> NewRow(),"Attitude Gains");
		alpha1_roll = new DoubleSpinBox(AttitudeGainsGroupBox->NewRow(),"Kp_roll:",0,10,0.001,4);
		alpha2_roll = new DoubleSpinBox(AttitudeGainsGroupBox->LastRowLastCol(),"Kd_roll:",0,10,0.001,4);

		alpha1_pitch = new DoubleSpinBox(AttitudeGainsGroupBox->NewRow(),"Kp_pitch:",0,10,0.001,4);
		alpha2_pitch = new DoubleSpinBox(AttitudeGainsGroupBox->LastRowLastCol(),"Kd_pitch:",0,10,0.001,4);

		alpha1_yaw = new DoubleSpinBox(AttitudeGainsGroupBox->NewRow(),"Kp_yaw:",0,10,0.001,4);
		alpha2_yaw = new DoubleSpinBox(AttitudeGainsGroupBox->LastRowLastCol(),"Kd_yaw:",0,10,0.001,4);
		
    GroupBox* PositionGainsGroupBox = new GroupBox(GainsSingle -> LastRowLastCol(),"Position Gains");
		a1 = new DoubleSpinBox(PositionGainsGroupBox->NewRow(),"a1:",0,10000,0.01,2);
		a2 = new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"a2:",0,10000,0.01,2);
		a3 = new DoubleSpinBox(PositionGainsGroupBox->NewRow(),"a3:",0,10000,0.01,2);
		a4 = new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"a4:",0,10000,0.01,2);
		a5 = new DoubleSpinBox(PositionGainsGroupBox->NewRow(),"a5:",0,10000,0.01,2);
    
		alpha1_x = new DoubleSpinBox(PositionGainsGroupBox->NewRow(),"Kp_x:",0,10,0.001,4);
		alpha2_x = new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"Kd_x:",0,10,0.001,4);
		alpha3_x = new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"Ki_x:",0,10,0.001,4);

		alpha1_y = new DoubleSpinBox(PositionGainsGroupBox->NewRow(),"Kp_y:",0,10,0.001,4);
		alpha2_y = new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"Kd_y:",0,10,0.001,4);
		alpha3_y= new DoubleSpinBox(PositionGainsGroupBox->LastRowLastCol(),"Ki_y:",0,10,0.001,4);
		
		Ux_gain= new DoubleSpinBox(PositionGainsGroupBox -> NewRow(),"Ux_Gain : ",0,1,0.010,3);
		Uy_gain = new DoubleSpinBox(PositionGainsGroupBox -> NewRow(),"Uy_Gain : ",0,1,0.010,3);

    
}


Controller4_1Class::~Controller4_1Class(void){
}

//void Law::UseDefaultPlot(const gui::LayoutPosition* position) {
void Controller4_1Class::UseDefaultPlot(const gui::LayoutPosition* position) {
   DataPlot1D *plot=new DataPlot1D(position,ObjectName(),-1,1);

   plot->AddCurve(state->Element(6,0),DataPlot::Red,"roll");
   plot->AddCurve(state->Element(7),DataPlot::Blue,"ref_roll");
   plot->AddCurve(state->Element(8),DataPlot::Red,"pitch");
   plot->AddCurve(state->Element(9),DataPlot::Blue,"ref_pitch");
   plot->AddCurve(state->Element(10),DataPlot::Red,"yaw");
   plot->AddCurve(state->Element(11),DataPlot::Blue,"ref_yaw");


}
/*void Controller2Class::UseDefaultPlot1(const gui::LayoutPosition* position) {
   DataPlot1D *plot1=new DataPlot1D(position,ObjectName(),-1,1);

   plot1->AddCurve(state->Element(6,0),DataPlot::Red,"roll");
   plot1->AddCurve(state->Element(7),DataPlot::Blue,"ref_roll");
   plot1->AddCurve(state->Element(8),DataPlot::Red,"pitch");
   plot1->AddCurve(state->Element(9),DataPlot::Blue,"ref_pitch");
   plot1->AddCurve(state->Element(10),DataPlot::Red,"yaw");
   plot1->AddCurve(state->Element(11),DataPlot::Blue,"ref_yaw");


}*/

//void Law::Reset(void) {		// casi creo que se puede borrar debido a que no tiene nada
void Controller4_1Class::Reset(void) {
}


//void Law::UpdateFrom(const io_data *data) {
void Controller4_1Class::UpdateFrom(const io_data *data) {

        input->GetMutex();
        Quaternion actualQuaternion(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
        Vector3Df actualOmega(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0));
        Vector3Df QuadPosition(input -> ValueNoMutex(7,0), input -> ValueNoMutex(8,0), input -> ValueNoMutex(9,0));
        Vector3Df QuadVel(input-> ValueNoMutex(10,0), input -> ValueNoMutex(11,0), input -> ValueNoMutex(12,0));
        int OperationSelector = input -> ValueNoMutex(13,0);
        int InitialLoop = input->ValueNoMutex(14,0);
   
        input->ReleaseMutex();

        Euler actualAttitude=actualQuaternion.ToEuler();
        
        cout << "You are in Controller Class 4_1 \n";
        
        //----------------------------->
        //		Control de Posición
        //----------------------------->
        Vector3Df PosErr;
        Vector3Df VelErr;
        Vector3Df IntErr;
        float Ux, Uy;
        float desiredPosx, desiredPosy;
        
        if (OperationSelector == 0)
        {
			desiredPosx = homePosx->Value();
			desiredPosy = homePosy->Value();
		}
		else
		{
			desiredPosx = userPosx ->Value();
			desiredPosy = userPosy ->Value();
		}
        
			cout << "\nDesiredPosition, x: " << desiredPosx<< "   y: " << desiredPosy<< endl;
			cout << "QuadPosition, x: " << QuadPosition.x <<"   y: " << QuadPosition.y << endl;
			//Error Posición
			PosErr.x = -QuadPosition.x + desiredPosx;
			PosErr.y = -QuadPosition.y + desiredPosy;
			//cout << "Pos Error, x: " << PosErr.x <<"   y: " << PosErr.y << endl;
			
			//Error Velocidad
			VelErr.x = -QuadVel.x + 0.0;
			VelErr.y = -QuadVel.y + 0.0;
			//cout << "Velocidad del quad, x = " << QuadVel.x << "   y = " << QuadVel.y << endl;
			//cout << "Error de Velocidad x = " << VelErr.x << "   y = "  << VelErr.y << endl;

			// Integral del error 
			
			//IntErr.x = NumericalIntegrationError(InitialLoop,PosErr.x);
			//IntErr.y = NumericalIntegrationError(InitialLoop,PosErr.y);
			
			//Controllers compute
			Ux = (alpha1_x->Value()*PosErr.x + alpha2_x->Value()*VelErr.x);// + alpha3_x->Value()*IntErr.x;
			Uy = (alpha1_y->Value()*PosErr.y + alpha2_y->Value()*VelErr.y);// + alpha3_y->Value()*IntErr.y;
					
			Ux = tanh(Ux);
			Uy = tanh(Uy);
			
			//cout << "Ux : " << Ux << "   Uy : " << Uy << endl;
						
        // -------------------------------->
        //		Computing the reference angles
        //--------------------------------->
        float roll_ref, pitch_ref;
        float argumentoRollRef;
        
			pitch_ref = asin(-Ux);
			
			argumentoRollRef = (Uy/cos(pitch_ref));
							
			roll_ref = asin(argumentoRollRef);
			
			pitch_ref = Ux_gain->Value()*tanh(pitch_ref);
			roll_ref  = Uy_gain->Value()*tanh(roll_ref);
        
        Euler AttitudeError;
        AttitudeError.roll=actualAttitude.roll-roll_ref;
        AttitudeError.pitch=actualAttitude.pitch-pitch_ref;
        AttitudeError.yaw=actualAttitude.yaw-0.0;
        
        Vector3Df OmegaError;
        OmegaError.x=actualOmega.x-0.0;
        OmegaError.y=actualOmega.y-0.0;
        OmegaError.z=actualOmega.z-0.0;
        
        Vector3Df P;
        P.x=alpha1_roll->Value()*AttitudeError.roll ;
        P.y=alpha1_pitch->Value()*AttitudeError.pitch;
        P.z=alpha1_yaw->Value()*AttitudeError.yaw;
        
        Vector3Df D;
        D.x=alpha2_roll->Value()*OmegaError.x ;
        D.y=alpha2_pitch->Value()*OmegaError.y;
        D.z=alpha2_yaw->Value()*OmegaError.z;
        
        Vector3Df NLTerms;
		NLTerms.x = -a1->Value()*actualOmega.y*actualOmega.z + a2->Value()*actualOmega.y;
		NLTerms.y = -a3->Value()*actualOmega.x*actualOmega.z + a4->Value()*actualOmega.x;
		NLTerms.z = -a5->Value()*actualOmega.x*actualOmega.y;

        Vector3Df torques;
        torques.x=P.x + D.x + NLTerms.x;
        torques.y=P.y + D.y + NLTerms.y;
        torques.z=P.z + D.z + NLTerms.z;
         
        state->GetMutex();
			
		state->SetValueNoMutex(0,0,QuadPosition.x);
		state->SetValueNoMutex(1,0,QuadPosition.y);
		state->SetValueNoMutex(2,0,QuadPosition.z);
		
		state->SetValueNoMutex(3,0,homePosx->Value());
		state->SetValueNoMutex(4,0,homePosy->Value());
		
		state->SetValueNoMutex(5,0,actualAttitude.roll);
		state->SetValueNoMutex(6,0,actualAttitude.pitch);
		state->SetValueNoMutex(7,0,actualAttitude.yaw);
		
		state->SetValueNoMutex(8,0,pitch_ref);
		state->SetValueNoMutex(9,0,roll_ref);
	
        
        
        state->ReleaseMutex();
        
        output->SetValue(0,0,torques.x);
        output->SetValue(1,0,torques.y);
        output->SetValue(2,0,torques.z);
        output->SetDataTime(data->DataTime());

        ProcessUpdate(output);
}

float Controller4_1Class::NumericalIntegrationError(int InitialLoop, float PositionError){
	
	actualTime = GetTime();
	cout << "acutual_time: " << actualTime << endl;
	delta_time = (float)(actualTime - previousTime)/1000000000;
	cout << "Delta time: " << delta_time << endl;
	delta_error = previousError - PositionError;	
	cout << "Delta_error: " << delta_error << endl;
	
	if (InitialLoop == 1 ){
		delta_time = 0;
		delta_error = 0;
		IntegralError = 0;
		cout << "Se ha reseteado el integrador" << endl;
	}
		
	IntegralError = IntegralError + (delta_error)*(delta_time)/2;
	cout << "IntegralError: " << IntegralError << endl;
		
	previousError = PositionError;
	previousTime = actualTime;
	cout << "PreviousTime: " << previousTime << endl;
		
	return IntegralError;

}
	


//void Law::SetValues(const AhrsData *actualOrientation,const AhrsData *referenceOrientation) {
void Controller4_1Class::SetValues(const core::AhrsData *actualOrientation, const core::Vector3Df uav_pos, const core::Vector3Df uav_vel,int ModeOperation, int *ResetLoop){
    Quaternion actualQuaternion;
    Vector3Df actualOmega, VelQuad;
    actualOrientation->GetQuaternionAndAngularRates(actualQuaternion,actualOmega);
    Vector3Df PositionQuad = uav_pos;
    int  OperationSelector = ModeOperation;

    input->SetValue(0,0,actualQuaternion.q0);
    input->SetValue(1,0,actualQuaternion.q1);
    input->SetValue(2,0,actualQuaternion.q2);
    input->SetValue(3,0,actualQuaternion.q3);
    input->SetValue(4,0,actualOmega.x);
    input->SetValue(5,0,actualOmega.y);
    input->SetValue(6,0,actualOmega.z);
    input->SetValue(7,0,PositionQuad.x);
    input->SetValue(8,0,PositionQuad.y);
    input->SetValue(9,0,PositionQuad.z);
    input->SetValue(10,0,VelQuad.x);
    input->SetValue(11,0,VelQuad.y);
    input->SetValue(12,0,VelQuad.z);  
    input->SetValue(13,0,OperationSelector);
    input->SetValue(14,0,*ResetLoop);

}

} // end namespace filter
} // end namespace flair
