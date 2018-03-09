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



#include "Controller5_1Class.h"
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
#include <CheckBox.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace std;

namespace flair { namespace filter {


Controller5_1Class::Controller5_1Class(TabWidget* tabwidget,string name) :  ControlLaw(tabwidget,name,3) {
	
	
	
	Tab *Cosensus = new Tab(tabwidget,"FormationTab");
	TabWidget *ConsensusLT = new TabWidget(Cosensus->NewRow(),"Tabs");
	//FormationSelector = new Tab(ConsensusLT,"Formation Selector");
	Formation1 = new Tab(ConsensusLT,"Formation1-A");
	Formation2 = new Tab(ConsensusLT,"Formation2"); 
	Formation3 = new Tab(ConsensusLT,"Formation3");
	Gains = new Tab(ConsensusLT,"Gains");
  
    input=new cvmatrix(this,33,1,floatType,name);
    
		cvmatrix_descriptor* desc=new cvmatrix_descriptor(33,1);
        desc->SetElementName(0,0,"x_0");
        desc->SetElementName(1,0,"x_1");
        desc->SetElementName(2,0,"x_2");
        
        desc->SetElementName(3,0,"y_1");
        desc->SetElementName(4,0,"y_2");
        desc->SetElementName(5,0,"y_3");

        desc->SetElementName(6,0,"z_1");
        desc->SetElementName(7,0,"z_2");
        desc->SetElementName(8,0,"z_3");
               
    	state=new cvmatrix(this,desc,floatType,name);
        AddDataToLog(state);

    Reset();
 
     // Regu Gain Values Spinboxes
    
    GroupBox* DegreeMatrixA = new GroupBox(Formation1->At(0,0),"Introducir la matriz de grado");
		DA0[0] = new DoubleSpinBox(DegreeMatrixA -> At(0,0),"D00: ",-5,5,0.01,3);
		DA0[1] = new DoubleSpinBox(DegreeMatrixA -> At(0,1),"D01: ",-5,5,0.01,3);
		DA0[2] = new DoubleSpinBox(DegreeMatrixA -> At(0,2),"D02: ",-5,5,0.01,3);

		DA1[0] = new DoubleSpinBox(DegreeMatrixA -> At(1,0),"D10: ",-5,5,0.01,3);
		DA1[1] = new DoubleSpinBox(DegreeMatrixA -> At(1,1),"D11: ",-5,5,0.01,3);
		DA1[2] = new DoubleSpinBox(DegreeMatrixA -> At(1,2),"D12: ",-5,5,0.01,3);
		
		DA2[0] = new DoubleSpinBox(DegreeMatrixA -> At(2,0),"D20: ",-5,5,0.01,3);
		DA2[1] = new DoubleSpinBox(DegreeMatrixA -> At(2,1),"D21: ",-5,5,0.01,3);
		DA2[2] = new DoubleSpinBox(DegreeMatrixA -> At(2,2),"D22: ",-5,5,0.01,3);
	
	GroupBox* AdjacencymatrixA = new GroupBox(Formation1->At(1,0),"Introducir la matriz de Adyacencia");
		AA0[0] = new DoubleSpinBox(AdjacencymatrixA -> At(3,0),"A00: ",-5,5,0.01,3);
		AA0[1] = new DoubleSpinBox(AdjacencymatrixA -> At(3,1),"A01: ",-5,5,0.01,3);
		AA0[2] = new DoubleSpinBox(AdjacencymatrixA -> At(3,2),"A02: ",-5,5,0.01,3);

		AA1[0] = new DoubleSpinBox(AdjacencymatrixA -> At(4,0),"A10: ",-5,5,0.01,3);
		AA1[1] = new DoubleSpinBox(AdjacencymatrixA -> At(4,1),"A11: ",-5,5,0.01,3);
		AA1[2] = new DoubleSpinBox(AdjacencymatrixA -> At(4,2),"A12: ",-5,5,0.01,3);
		
		AA2[0] = new DoubleSpinBox(AdjacencymatrixA -> At(5,0),"A20: ",-5,5,0.01,3);
		AA2[1] = new DoubleSpinBox(AdjacencymatrixA -> At(5,1),"A21: ",-5,5,0.01,3);
		AA2[2] = new DoubleSpinBox(AdjacencymatrixA -> At(5,2),"A22: ",-5,5,0.01,3);
		
	GroupBox* FleetParams = new GroupBox(Formation1->At(1,1),"Fleet Parameters");
		FleetXPos = new DoubleSpinBox(FleetParams -> At(0,0),"FleetPos X: ",-5,5,0.01,3);
		GainLeaderX = new DoubleSpinBox(FleetParams -> At(0,1),"Gain Fleet X: ",-5,5,0.01,3);
		FleetYPos = new DoubleSpinBox(FleetParams -> At(1,0),"FleetPos Y: ",-5,5,0.01,3);
		GainLeaderY = new DoubleSpinBox(FleetParams -> At(1,1),"Gain Fleet Y: ",-5,5,0.01,3);
		
	GroupBox* LeaderSelector = new GroupBox(Formation1 -> At(2,0), "Leader Selector");
		Selector00 = new CheckBox(LeaderSelector -> At(0,0), "Drone x8_0",0);
		Selector01 = new CheckBox(LeaderSelector -> At(0,0), "Drone x8_1",0);
		Selector02 = new CheckBox(LeaderSelector -> At(0,0), "Drone x8_2",0);
		
		
		
	GroupBox* AttitudeGains = new GroupBox(Gains->At(0,0),"Attitude Gains");
		kp_roll = new DoubleSpinBox(AttitudeGains->NewRow(),"Kp_roll:",0,10,0.001,4);
		kd_roll = new DoubleSpinBox(AttitudeGains->LastRowLastCol(),"Kd_roll:",0,10,0.001,4);

		kp_pitch = new DoubleSpinBox(AttitudeGains->NewRow(),"Kp_pitch:",0,10,0.001,4);
		kd_pitch = new DoubleSpinBox(AttitudeGains->LastRowLastCol(),"Kd_pitch:",0,10,0.001,4);

		kp_yaw = new DoubleSpinBox(AttitudeGains->NewRow(),"Kp_yaw:",0,10,0.001,4);
		kd_yaw = new DoubleSpinBox(AttitudeGains->LastRowLastCol(),"Kd_yaw:",0,10,0.001,4);
		
		kd_x_consensus = new DoubleSpinBox(AttitudeGains->NewRow(),"alpha_x_cons: ",-10,10,0.000001,6);
		kd_y_consensus = new DoubleSpinBox(AttitudeGains->NewRow(),"alpha_y_cons: ",-10,10,0.000001,6); 
	
	GroupBox* ConsensusGainsSPGroup = new GroupBox(Gains->At(0,1),"Consensus Gains");
		ConsensusGains[0] = new DoubleSpinBox(ConsensusGainsSPGroup->At(0,0),"c00: ",-10,10,0.001,4);
		ConsensusGains[1] = new DoubleSpinBox(ConsensusGainsSPGroup->At(0,1),"c01: ",-10,10,0,001,4);
		ConsensusGains[2] = new DoubleSpinBox(ConsensusGainsSPGroup->At(0,2),"c02: ",-10,10,0,001,4);
		ConsensusGains[3] = new DoubleSpinBox(ConsensusGainsSPGroup->At(1,0),"c10: ",-10,10,0,001,4);
		ConsensusGains[4] = new DoubleSpinBox(ConsensusGainsSPGroup->At(1,1),"c11: ",-10,10,0,001,4);
		ConsensusGains[5] = new DoubleSpinBox(ConsensusGainsSPGroup->At(1,2),"c12: ",-10,10,0,001,4);
		ConsensusGains[6] = new DoubleSpinBox(ConsensusGainsSPGroup->At(2,0),"c20: ",-10,10,0,001,4);
		ConsensusGains[7] = new DoubleSpinBox(ConsensusGainsSPGroup->At(2,1),"c21: ",-10,10,0,001,4);
		ConsensusGains[8] = new DoubleSpinBox(ConsensusGainsSPGroup->At(2,2),"c22: ",-10,10,0,001,4);

	GroupBox* ScaleGroupBox = new GroupBox(Gains->At(1,0),"Scale Control");
		ux = new DoubleSpinBox(ScaleGroupBox -> NewRow(),"Ux_Gain: ",0,2,0.001,4);
		uy = new DoubleSpinBox(ScaleGroupBox -> NewRow(),"Uy_Gain: ",0,2,0.001,4);
	
/*	
		a1 = new DoubleSpinBox(ConsensusGains->NewRow(),"a1:",0,10000,0.01,2);
		a2 = new DoubleSpinBox(ConsensusGains->LastRowLastCol(),"a2:",0,10000,0.01,2);
		a3 = new DoubleSpinBox(ConsensusGains->NewRow(),"a3:",0,10000,0.01,2);
		a4 = new DoubleSpinBox(ConsensusGains->LastRowLastCol(),"a4:",0,10000,0.01,2);
		a5 = new DoubleSpinBox(ConsensusGains->NewRow(),"a5:",0,10000,0.01,2);
    
		kp_x = new DoubleSpinBox(ConsensusGains->NewRow(),"Kp_x:",0,10,0.001,4);
		kd_x = new DoubleSpinBox(ConsensusGains->LastRowLastCol(),"Kd_x:",0,10,0.001,4);

		kp_y = new DoubleSpinBox(ConsensusGains->NewRow(),"Kp_y:",0,10,0.001,4);
		kd_y = new DoubleSpinBox(ConsensusGains->LastRowLastCol(),"Kd_y:",0,10,0.001,4);
		
		ux = new DoubleSpinBox(ConsensusGains -> NewRow(),"Ux_Gain : ",0,1,0.010,3);
		uy = new DoubleSpinBox(ConsensusGains -> NewRow(),"Uy_Gain : ",0,1,0.010,3);
*/
		
		
		
		/*****************************		Formaciones  		*/
	GroupBox* Formacion00 = new GroupBox(Formation2->At(0,0),"Structure1");
		  
		RelDistx[0] = new DoubleSpinBox(Formacion00 -> At(0,0),"0->1 x0: ",-5,5,0.01,2);
		RelDistx[1] = new DoubleSpinBox(Formacion00 -> At(1,0),"0->2 x0: ",-5,5,0.01,2);
		RelDistx[2] = new DoubleSpinBox(Formacion00 -> At(2,0),"1->2 x0: ",-5,5,0.01,2);
		
		RelDisty[0] = new DoubleSpinBox(Formacion00 -> At(0,1),"0->1 y0: ",-5,5,0.01,4);
		RelDisty[1] = new DoubleSpinBox(Formacion00 -> At(1,1),"0->2 y0: ",-5,5,0.01,4);
		RelDisty[2] = new DoubleSpinBox(Formacion00 -> At(2,1),"1->2 y0: ",-5,5,0.01,4);
		
	GroupBox* FormationB = new GroupBox(Formation2->At(0,1),"Structure2");
		RelDistx[3] = new DoubleSpinBox(FormationB -> At(0,0),"0->1 x1: ",-5,5,0.01,3);
		RelDistx[4] = new DoubleSpinBox(FormationB -> At(1,0),"0->2 x1: ",-5,5,0.01,3);
		RelDistx[5] = new DoubleSpinBox(FormationB -> At(2,0),"1->2 x1: ",-5,5,0.01,3);
		
		RelDisty[3] = new DoubleSpinBox(FormationB -> At(0,1),"0->1 y1: ",-5,5,0.01,3);
		RelDisty[4] = new DoubleSpinBox(FormationB -> At(1,1),"0->2 y1: ",-5,5,0.01,3);
		RelDisty[5] = new DoubleSpinBox(FormationB -> At(2,1),"1->2 y1: ",-5,5,0.01,3);
		
	GroupBox* FormationC = new GroupBox(Formation2->At(0,2),"Structure3");
		RelDistx[6] = new DoubleSpinBox(FormationC -> At(0,0),"0->1 x2: ",-5,5,0.01,3);
		RelDistx[7] = new DoubleSpinBox(FormationC -> At(1,0),"0->2 x2: ",-5,5,0.01,3);
		RelDistx[8] = new DoubleSpinBox(FormationC -> At(2,0),"1->2 x2: ",-5,5,0.01,3);
		
		RelDisty[6] = new DoubleSpinBox(FormationC -> At(0,1),"0->1 y2: ",-5,5,0.01,3);
		RelDisty[7] = new DoubleSpinBox(FormationC -> At(1,1),"0->2 y2: ",-5,5,0.01,3);
		RelDisty[8] = new DoubleSpinBox(FormationC -> At(2,1),"1->2 y2: ",-5,5,0.01,3);
		
	GroupBox* FormationSelector = new GroupBox(Formation2->At(1,1),"Formation Selector");
		FFlightSelector  = new DoubleSpinBox(FormationSelector->At(0,0),"Formation Selector: ", 0,2,1,0);

		

	
    
}


Controller5_1Class::~Controller5_1Class(void){
}

//void Law::UseDefaultPlot(const gui::LayoutPosition* position) {
void Controller5_1Class::UseDefaultPlot(const gui::LayoutPosition* position) {
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
void Controller5_1Class::Reset(void) {
}


//void Law::UpdateFrom(const io_data *data) {
void Controller5_1Class::UpdateFrom(const io_data *data) {
        input->GetMutex();
        Quaternion actualQuaternion(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
        Vector3Df actualOmega(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0));
        float StatesXdr[3],StatesYdr[3],StatesZdr[3]; 
        float UxFormationControl, UyFormationControl;    
        float FlightFormationX, FlightFormationY;  
        for( int i = 0; i < 3 ; i++ ){
			StatesXdr[i] = input->ValueNoMutex(7+i,0);
			StatesYdr[i] = input->ValueNoMutex(10+i,0);
			StatesZdr[i] = input->ValueNoMutex(17+i,0);
		}
		int IdDrone = input->ValueNoMutex(13,0);
		Vector3Df VelQuad(input->ValueNoMutex(14,0),input->ValueNoMutex(15,0),input->ValueNoMutex(16,0));
		
		
		
		float UxConsensusCtrl, UyConsensusCtrl;
		
//		cout << endl;
   
        input->ReleaseMutex();

        Euler actualAttitude=actualQuaternion.ToEuler();
        
 //       cout << "You are in Controller5class \n";
        
        state->GetMutex();
			
		state->SetValueNoMutex(0,0,StatesXdr[0]);
		state->SetValueNoMutex(1,0,StatesXdr[1]);
		state->SetValueNoMutex(2,0,StatesXdr[2]);
		
		state->SetValueNoMutex(3,0,StatesYdr[0]);
		state->SetValueNoMutex(4,0,StatesYdr[1]);
		state->SetValueNoMutex(5,0,StatesYdr[2]);
		
		state->SetValueNoMutex(6,0,StatesZdr[0]);
		state->SetValueNoMutex(7,0,StatesZdr[1]);
		state->SetValueNoMutex(8,0,StatesZdr[2]);
		
        state->ReleaseMutex();
 //       cout << "Fin de GetMutex \n ";
        
        
        DegreeRead();
        AdjacencyRead();
        CreateLaplacian();
        
        int IdLeader;
          
        /**************************************************************
         *		Diseño del Control MultiAgente						  *
         * ***********************************************************/
        
        // Leader Selector
		
		if( Selector00->Value() == 1 )
			IdLeader = 0;
		else if (Selector01->Value() == 1)
			IdLeader = 1;
		else if (Selector02->Value() == 1)
			IdLeader = 2;
		

		RelativeDistancesVector(IdDrone);
		
		ConsensusGainsVector(IdDrone);
		
		
		
//		cout << "El lider es el drone " << IdLeader << endl;
			
		switch (IdDrone){
			
			
			case 0: 			
				UxLeaderCtrl = Selector00->Value()*GainLeaderX->Value()*(StatesXdr[0]-FleetXPos->Value());
				UyLeaderCtrl = Selector00->Value()*GainLeaderY->Value()*(StatesYdr[0]-FleetYPos->Value());
				FlightFormationX = FlightFormationProtocol(&StatesXdr[0],&VecAA0[0],&VecConsensusGains[0],&VecRelDistx[0],IdDrone);
				FlightFormationY = FlightFormationProtocol(&StatesYdr[0],&VecAA0[0],&VecConsensusGains[0],&VecRelDisty[0],IdDrone);
				UxTotalCtrl = -UxLeaderCtrl - FlightFormationX + kd_x_consensus->Value()*VelQuad.x ;
				UyTotalCtrl = -UyLeaderCtrl - FlightFormationY + kd_y_consensus->Value()*VelQuad.y;
//				cout << "Error 0->1 x: "<< StatesXdr[0]-StatesXdr[1] << "  Error 0->1 y: " << StatesYdr[0] - StatesYdr[1] << endl;
				break;
				
			case 1:
				UxLeaderCtrl = Selector01->Value()*GainLeaderX->Value()*(StatesXdr[1]-FleetXPos->Value());
				UyLeaderCtrl = Selector01->Value()*GainLeaderY->Value()*(StatesYdr[1]-FleetYPos->Value());
				FlightFormationX = FlightFormationProtocol(&StatesXdr[0],&VecAA1[0],&VecConsensusGains[0],&VecRelDistx[0],IdDrone);		
				FlightFormationY = FlightFormationProtocol(&StatesYdr[0],&VecAA1[0],&VecConsensusGains[0],&VecRelDisty[0],IdDrone);		
				UxTotalCtrl = -UxLeaderCtrl - FlightFormationX + kd_x_consensus->Value()*VelQuad.x ;;
				UyTotalCtrl = -UyLeaderCtrl - FlightFormationY + kd_y_consensus->Value()*VelQuad.y;;
//				cout << "Error 0->1 x: "<< StatesXdr[1]-StatesXdr[2] << "  Error 0->1 y: " << StatesYdr[1] - StatesYdr[2] << endl;

				break;
				
			case 2:
				UxLeaderCtrl = Selector02->Value()*GainLeaderX->Value()*(StatesXdr[2]-FleetXPos->Value());
				UyLeaderCtrl = Selector02->Value()*GainLeaderY->Value()*(StatesYdr[2]-FleetYPos->Value());
				FlightFormationX = FlightFormationProtocol(&StatesXdr[0],&VecAA2[0],&VecConsensusGains[0],&VecRelDistx[0],IdDrone);				
				FlightFormationY = FlightFormationProtocol(&StatesYdr[0],&VecAA2[0],&VecConsensusGains[0],&VecRelDisty[0],IdDrone);
				UxTotalCtrl = -UxLeaderCtrl - FlightFormationX + kd_x_consensus->Value()*VelQuad.x ;;
				UyTotalCtrl = -UyLeaderCtrl - FlightFormationY + kd_y_consensus->Value()*VelQuad.y;;	
//				cout << "Error 2->1 x: "<< StatesXdr[2]-StatesXdr[1] << "  Error 2->1 y: " << StatesYdr[2] - StatesYdr[1] << endl;				
				break;
			default :
				cout << endl << "No mamars, eso no puede ser, seguro ya te cagaste fuera del hoyo" << endl;
			
		}
		
//		cout << "UxTotalCtrl: " << UxTotalCtrl << "   UyTotalCtrl: " << UyTotalCtrl << endl;
//		cout << "UxTotalTanh: " << tanh(UxTotalCtrl) << "   UyTotalTanh: " << tanh(UyTotalCtrl)<< endl;
		
		
	
		
		UxTotalCtrl = 0.3*tanh(UxTotalCtrl);
		UyTotalCtrl = 0.3*tanh(UyTotalCtrl);
		
//		cout << endl << "Las señales de control son, Ux: " << UxTotalCtrl << "   Uy: " << UyTotalCtrl << endl;

		
		float roll_ref, pitch_ref;
        float argumentoRollRef;
        
			pitch_ref = asin(-UxTotalCtrl);
			
			argumentoRollRef = tanh( (UyTotalCtrl)/cos(pitch_ref));
//			cout << "ArgumentoRollRef : " << argumentoRollRef << endl;
							
			roll_ref = asin(argumentoRollRef);
	

				
			pitch_ref = ux->Value()*tanh(pitch_ref);
			roll_ref  = uy->Value()*tanh(roll_ref);			
			
//		cout << "Ref Pitch: " << pitch_ref << "   Ref Roll: " << roll_ref << endl; 
	
        
        Euler AttitudeError;
        
//		cout << "Generando los errores proporcionales de orientacion\n";
        AttitudeError.roll=actualAttitude.roll-roll_ref;
        AttitudeError.pitch=actualAttitude.pitch-pitch_ref;
        AttitudeError.yaw=actualAttitude.yaw-0.0;
        
//        cout << "Error Roll: " << AttitudeError.roll << "  Error Pitch: " << AttitudeError.pitch << "   Error Yaw: " << AttitudeError.yaw << endl;
        
        Vector3Df OmegaError;
        
//        cout << "Generando los derivativos de orientacion\n";
        OmegaError.x=actualOmega.x-0.0;
        OmegaError.y=actualOmega.y-0.0;
        OmegaError.z=actualOmega.z-0.0;


       Vector3Df torques;
       torques.x = kp_roll->Value()*AttitudeError.roll + kd_roll->Value()*OmegaError.x;
       torques.y = kp_pitch->Value()*AttitudeError.pitch + kd_pitch->Value()*OmegaError.y;
       torques.z = kp_yaw->Value()*AttitudeError.yaw + kd_yaw->Value()*OmegaError.z;
         
//		cout << "torque.x : " << torques.x << endl;
//		cout << "torque.y : " << torques.y << endl;
//		cout << "torque.z : " << torques.z << endl;


       state->GetMutex();
			
		state->SetValueNoMutex(0,0,StatesXdr[0]);
		state->SetValueNoMutex(1,0,StatesXdr[1]);
		state->SetValueNoMutex(2,0,StatesXdr[2]);
		
		state->SetValueNoMutex(3,0,StatesXdr[0]);
		state->SetValueNoMutex(4,0,StatesXdr[1]);		
		state->SetValueNoMutex(5,0,StatesXdr[3]);

		state->SetValueNoMutex(6,0,StatesZdr[0]);
		state->SetValueNoMutex(7,0,StatesZdr[1]);
		state->SetValueNoMutex(7,0,StatesZdr[2]);
     
        state->ReleaseMutex();

        output->SetValue(0,0,torques.x);
        output->SetValue(1,0,torques.y);
        output->SetValue(2,0,torques.z);
        output->SetDataTime(data->DataTime());

        previous_time=data->DataTime();
        ProcessUpdate(output);
}


float Controller5_1Class::FlightFormationProtocol(float *ptrStates, float *ptrVecAdjacency, float *VecGainsProtocol, float *ptrRelativeDistances, int IdDrone)
{
	float U_protocol = 0;
	float u0,u1,u2;
//	cout <<"States: " << *(ptrStates + 0) << "   " << *(ptrStates + 1) << "   " << *(ptrStates + 2) << endl;
//	cout <<"Adjacency: " << *(ptrVecAdjacency + 0) << "   " << *(ptrVecAdjacency + 1) << "   " << *(ptrVecAdjacency + 2) << endl;
//	cout <<"GainsProtocol: " << *(VecGainsProtocol + 0) << "   " << *(VecGainsProtocol + 1) << "   " << *(VecGainsProtocol + 2) << endl;
//	cout <<"RelDistances: " << *(ptrRelativeDistances + 0) << "   " << *(ptrRelativeDistances + 1) << "   " << *(ptrRelativeDistances + 2) << endl;
//	cout <<"IdDrone: " << IdDrone << endl;
	u0 = *(ptrVecAdjacency + 0)*(*(VecGainsProtocol+0))*((*(ptrStates+IdDrone)-*(ptrStates+0))-*(ptrRelativeDistances+0));
	u1 = *(ptrVecAdjacency + 1)*(*(VecGainsProtocol+1))*((*(ptrStates+IdDrone)-*(ptrStates+1))-*(ptrRelativeDistances+1));
	u2 = *(ptrVecAdjacency + 2)*(*(VecGainsProtocol+2))*((*(ptrStates+IdDrone)-*(ptrStates+2))-*(ptrRelativeDistances+2));
	//for( int i = 0; i < 3 ; i++ )
	//{
	//U_protocol = U_protocol + (*(ptrVecAdjacency + i)*((*(ptrStates+IdDrone)-*(ptrStates+i))-*(ptrRelativeDistances+i)));
	//}
	
	U_protocol = u0 + u1+ u2;
	return U_protocol;
}

void Controller5_1Class::ConsensusGainsVector( int id_drone )
{
	
	int index;
	switch(id_drone)
	{
		case 0:
			index = 0;
			break;
		case 1:
			index = 3;
			break;
		case 2:
			index = 6;
			break;
		default:
			index = 0;
//			cout << "Error ----> Formation 0 by default" << endl;
			break;		
	}
	
//	cout << "Index for gains: " << index << endl;
	
	switch(id_drone)
	{
		case 0: 
			VecConsensusGains[0] = ConsensusGains[index + 0]->Value(); 
			VecConsensusGains[1] = ConsensusGains[index + 1]->Value();
			VecConsensusGains[2] = ConsensusGains[index + 2]->Value();
			break;
		
		case 1:
		
			VecConsensusGains[0] = ConsensusGains[index + 0]->Value(); 
			VecConsensusGains[1] = ConsensusGains[index + 1]->Value(); 
			VecConsensusGains[2] = ConsensusGains[index + 2]->Value(); 
			break;
		
		case 2:
			VecConsensusGains[0] = ConsensusGains[index + 0]->Value(); 
			VecConsensusGains[1] = ConsensusGains[index + 1]->Value(); 
			VecConsensusGains[2] = ConsensusGains[index + 2]->Value(); 
			
	}
	
	//cout << "Vector Gains: " << VecConsensusGains[0] << "   " << VecConsensusGains[1] << "   " << VecConsensusGains[2] << endl;
}


void Controller5_1Class::RelativeDistancesVector ( int id_drone )
{
	int index;
	switch((int)(FFlightSelector->Value()))
	{
		case 0:
			index = 0;
			break;
		case 1:
			index = 3;
			break;
		case 2:
			index = 6;
			break;
		default:
			index = 0;
		//	cout << "Error ----> Formation 0 by default" << endl;
			break;		
	}
	
//	cout << "Index: " << index << endl;
	
	switch(id_drone)
	{
		//Se han cambiado los signos 13:50p - 06/Noviembre/2017
		case 0: 
			VecRelDistx[0] = 0; 
			VecRelDistx[1] = -RelDistx[index]->Value();
			VecRelDistx[2] = -RelDistx[index + 1]->Value();
			
			VecRelDisty[0] = 0;
			VecRelDisty[1] = -RelDisty[index]->Value();
			VecRelDisty[2] = -RelDisty[index + 1]->Value();
			break;
		
		case 1:
		
			VecRelDistx[0] = RelDistx[index]->Value();
			VecRelDistx[1] = 0;
			VecRelDistx[2] = -RelDistx[index + 2]->Value();
			
			VecRelDisty[0] = RelDisty[index]->Value();
			VecRelDisty[1] = 0;
			VecRelDisty[2] = -RelDisty[index + 2]->Value();
			break;
		
		case 2:
			VecRelDistx[0] = RelDistx[index + 1]->Value();
			VecRelDistx[1] = RelDistx[index + 2]->Value();
			VecRelDistx[2] = 0;
			
			VecRelDisty[0] = RelDisty[index + 1]->Value();
			VecRelDisty[1] = RelDisty[index + 2]->Value();
			VecRelDisty[2] = 0;
			
	}
	
//	cout << "Vector relative distances: " << VecRelDistx[0] << "   " << VecRelDistx[1] << "   " << VecRelDistx[2] << endl;
//	cout << "Vector relative distances: " << VecRelDisty[0] << "   " << VecRelDisty[1] << "   " << VecRelDisty[2] << endl;
	
			
}



void Controller5_1Class::DegreeRead( void )
{
	
	for( int i = 0 ; i < 3 ; i++ )
	{
		*(ptrDA0+i) = DA0[i]->Value();
		*(ptrDA1+i) = DA1[i]->Value();
		*(ptrDA2+i) = DA2[i]->Value();
	}
}

void Controller5_1Class::AdjacencyRead( void )
{
	for( int i = 0 ; i < 3 ; i++ )
	{
		*(ptrAA0+i) = AA0[i]->Value();
		*(ptrAA1+i) = AA1[i]->Value();
		*(ptrAA2+i) = AA2[i]->Value();
	}
}

void Controller5_1Class::CreateLaplacian( void )
{
	for( int i = 0; i < 3 ; i++)
	{
		*(ptrLA0+i) = *(ptrDA0 + i) - *(ptrAA0 + i);
		*(ptrLA1+i) = *(ptrDA1 + i) - *(ptrAA1 + i);
		*(ptrLA2+i) = *(ptrDA2 + i) - *(ptrAA2 + i);
	}
}

float Controller5_1Class::DotProduct(float *ptrVector1, float *ptrVector2)
{
	float Result = 0;
	for(int i = 0; i < 3; i++ )
	{
		Result += *(ptrVector1+i)*(*(ptrVector2+i));
	}
	return Result;
}

//void Law::SetValues(const AhrsData *actualOrientation,const AhrsData *referenceOrientation) {
//void Controller3Class::SetValues(const core::AhrsData *actualOrientation, const core::Vector3Df uav_pos, const core::Vector3Df uav_vel,int ModeOperation){
void Controller5_1Class::SetValues(const core::AhrsData *actualOrientation, float *PtrStateX, float *PtrStateY, int *PtrIdDrone, core::Vector3Df uav_vel, float *PtrStateZ){

		//	Consensus -> SetValues(customOrientation, &StateX[0], &StateY[0], &IdDrone)
    Quaternion actualQuaternion;
    Vector3Df actualOmega;
    actualOrientation->GetQuaternionAndAngularRates(actualQuaternion,actualOmega);
    Vector3Df VelQuad = uav_vel;

    input->SetValue(0,0,actualQuaternion.q0);
    input->SetValue(1,0,actualQuaternion.q1);
    input->SetValue(2,0,actualQuaternion.q2);
    input->SetValue(3,0,actualQuaternion.q3);
    input->SetValue(4,0,actualOmega.x);
    input->SetValue(5,0,actualOmega.y);
    input->SetValue(6,0,actualOmega.z);
    
//    cout << endl;
    for(int i = 0; i < 3; i++ )
    {
		input->SetValue(7+i,0,*(PtrStateX + i));
//		cout << "x" << i << " : " << *(PtrStateX+i) << endl;
	}
	for(int i = 0; i < 3; i++ )
	{
		input->SetValue(10+i,0,*(PtrStateY + i));
//		cout << "y" << i << " : " << *(PtrStateY+i) << endl;
	}
	input->SetValue(13,0,*PtrIdDrone);
	input->SetValue(14,0,VelQuad.x);
    input->SetValue(15,0,VelQuad.y);
    input->SetValue(16,0,VelQuad.z);
    input->SetValue(17,0,*(PtrStateZ + 0));
    input->SetValue(18,0,*(PtrStateZ + 1));
    input->SetValue(19,0,*(PtrStateZ + 2));
	
		   
}

} // end namespace filter
} // end namespace flair
