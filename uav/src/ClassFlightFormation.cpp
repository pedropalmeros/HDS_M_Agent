//  created:    2015/11/05
//  filename:   SimpleFleet.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo fleet
//
//
/*********************************************************************/

#include "ClassFlightFormation.h"

#include "Controller5_1Class.h"
#include "Controller4_1Class.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <Ahrs.h>
#include <AhrsData.h> 
#include <MetaUsRangeFinder.h>
#include <MetaDualShock3.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Vector3D.h>
#include <Vector2D.h>
#include <PidThrust.h>
#include <Euler.h>
#include <cvmatrix.h>
#include <AhrsData.h>
#include <Ahrs.h>
#include <DoubleSpinBox.h>
#include <stdio.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <UdpSocket.h>
#include <string.h>
#include <iostream> 
#include <GroupBox.h>
#include <sstream>
#include <UdpSocket.h>
#include <DoubleSpinBox.h>
#include <Vector3DSpinBox.h>
#include <SpinBox.h>
#include <TabWidget.h>

#define PI ((float)3.14159265358979323846)
#define NoDrones 3

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


FormationFlightCore::FormationFlightCore(string broadcast,TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();
	
	//Optitrack	
	VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80);
	uavVrpn = new MetaVrpnObject(uav->ObjectName());
	getFrameworkManager()->AddDeviceToLog(uavVrpn);
	uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

	std::cout << "hola\n";
	std::cout << uav->ObjectName() << std::endl;
	Name_Drone = uav->ObjectName();
	std::cout << "El nombre del Drone es: " << Name_Drone << std::endl;	
	switch(Name_Drone[3]){
		case '0':
		{
			*ptrIdDrone = 0;
			cout << "El Id del Drone es: " << *ptrIdDrone << endl;
			break;
		}		
		case '1':
		{
			*ptrIdDrone = 1;
			cout << "El Id del Drone es: " << *ptrIdDrone << endl;			
			break;		
		}
		case '2':
		{
			*ptrIdDrone = 2;
			cout << "El Id del Drone es: " << *ptrIdDrone << endl;
			break;
		}
	}
	
	
	
	// En esta sección se crean los demás drones y se les asigna un nombre
	for(int i=0;i<3;i++) {
		stringstream s;
		s << "x4_" << i;
		//cout << s << endl;
		cout << s.str();   // Imprime la cadena del nuevo nombre del drone que está creando 
		if(s.str()==uav->ObjectName())
		{
			ArrayDrones[i] = uavVrpn;//self
			cout << " => if - Se ha creado el drone no. " << i << endl;;
		}
		else
		{
			ArrayDrones[i] = new MetaVrpnObject(s.str());//others
			cout << " => else - Se ha creado el drone no. " << i << endl;
			uavVrpn->XyPlot()->AddCurve(ArrayDrones[i]->Output()->Element(5,0),ArrayDrones[i]->Output()->Element(4,0));
		}
		cout << i << endl;
    }
	cout << "Fin de For";
   	
	vrpnclient->Start();
	
	StatesDrones();
	
	Tab *AllQuadPositions = new Tab(getFrameworkManager()->GetTabWidget(),"AllQuadPositions");
	TabWidget *AllQuadPositionsLT = new TabWidget(AllQuadPositions->NewRow(),"PositionControl");
	PositionTime = new Tab(AllQuadPositionsLT,"Positions");
		plotx1 = new DataPlot1D(PositionTime->At(0,0),"x0",-10,10);
	//		plotx1->AddCurve(PositionAllDrones[0].x,DataPlot::Red,"x001");
		plotx2 = new DataPlot1D(PositionTime->At(1,0),"x1",-10,10);
		plotx3 = new DataPlot1D(PositionTime->At(2,0),"x2",-10,10);
		
		plotx1 = new DataPlot1D(PositionTime->At(0,1),"y0",-10,10);
		plotx2 = new DataPlot1D(PositionTime->At(1,1),"y1",-10,10);
		plotx3 = new DataPlot1D(PositionTime->At(2,1),"y2",-10,10);
	Positionxy = new Tab(AllQuadPositionsLT,"Gains"); 
	

	cout << "Vrpn Client Started" << endl;
	
	StartBehaviour1Button = new PushButton(GetButtonsLayout() -> NewRow(), "Home");
	StartBehaviour2Button = new PushButton(GetButtonsLayout() -> LastRowLastCol(), "Independet Positions");
	StartBehaviour3Button = new PushButton(GetButtonsLayout() -> NewRow(), "Formation");
	StartBehaviour4Button = new PushButton(GetButtonsLayout() -> LastRowLastCol(), "Change to Behaviour 4");
	
	cout << "Botones creados" << endl;

	cout << "Constructor de la clase PositionNewProg"<< endl;
	Position = new Controller4_1Class(getFrameworkManager()->GetTabWidget(),"Position");
	//Position = new Controller4Class(getFrameworkManager()->GetTabWidget(),"Position");
	Formation = new Controller5_1Class(getFrameworkManager()->GetTabWidget(),"Formation");
	
	cvmatrix_descriptor* EstadosQuad = new cvmatrix_descriptor(9,1);
	EstadosQuad->SetElementName(0,0,"x0");
	EstadosQuad->SetElementName(1,0,"x1");
	EstadosQuad->SetElementName(2,0,"x2");
	EstadosQuad->SetElementName(3,0,"y0");
	EstadosQuad->SetElementName(4,0,"y1");
	EstadosQuad->SetElementName(5,0,"y2");
	EstadosQuad->SetElementName(6,0,"z0");
	EstadosQuad->SetElementName(7,0,"z1");
	EstadosQuad->SetElementName(8,0,"z2");
	
	EstadosQuadMatrix = new cvmatrix(this,EstadosQuad,floatType,"EstadosQuadMatrix");
	AddDataToControlLawLog(EstadosQuadMatrix);
	
	
	
    message=new UdpSocket(uav,"Message",broadcast,true);

    customReferenceOrientation= new AhrsData(this,"reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);

    customOrientation=new AhrsData(this,"orientation");
          


}

FormationFlightCore::~FormationFlightCore() {
}


void FormationFlightCore::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);

    switch(event) {
    case Event_t::EmergencyStop:
        message->SendMessage("EmergencyStop");
        cout << "Emergency Stop has been required" << endl;
        break;
    case Event_t::TakingOff:
        message->SendMessage("TakeOff");
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        cout << "Taking Off has been required" << endl;
        break;
    case Event_t::StartLanding:
		cout << "StartLanding has been required" << endl;
        message->SendMessage("Landing");
        break;
    case Event_t::EnteringControlLoop:
		//cout << "Entering ControlLoop has been required" << endl;
        CheckMessages();
        break;
    case Event_t::EnteringFailSafeMode:
		cout << "Entering Fail Safe Mode has been required" << endl;
        //StopLaw();
        break;
    }
}

void FormationFlightCore::CheckMessages(void) {
    char msg[64];
    char src[64];
    size_t src_size=sizeof(src);
    while(message->RecvMessage(msg,sizeof(msg),TIME_NONBLOCK,src,&src_size)>0) {
        //printf("%s %s\n",GetUav()->ObjectName().c_str(),src);
        if(strcmp(src,GetUav()->ObjectName().c_str())!=0) {
             if(strcmp(msg,"TakeOff")==0) {
                Printf("TakeOff fleet\n");
                cout << "TakeOff fleet\n";
                TakeOff();
                *ptrInitialLoop = 1;
            }
            if(strcmp(msg,"Landing")==0) {
                Printf("Landing fleet\n");
                cout << "Landing fleet\n" ;
                Land();
            }
            if(strcmp(msg,"EmergencyStop")==0) {
                Printf("EmergencyStop fleet\n");
                cout << "Emergency Stop fleet\n";
                EmergencyStop();
            }
            if(strcmp(msg,"Behaviour1")==0){
				//Printf("Behaviour1 Requested\n");
				cout << "Check Message => Behaviour1 Requested\n";
				Ch2Controller1Fnc();
				*ptrInitialLoop = 1;
			}
			if(strcmp(msg,"Behaviour2")==0){
				//Printf("Behaviour2 Requested\n");
				cout << "Check Message => Behaviour2 Requested \n";
				Ch2Controller2Fnc();	
				*ptrInitialLoop = 1;	
			}
			if(strcmp(msg,"Behaviour3")==0){
				//Printf("Behaviour3 Requested\n");
				cout << "Check Message => Behaviour3 Requested\n";
				Ch2Controller3Fnc();
			}
			if(strcmp(msg,"Behaviour4")==0){
				//Printf("Behaviour4 Requested\n");
				cout << "Check Message => Behaviour4 Requested \n";	
				Ch2Controller4Fnc();	
			}
			
        }
    }
}

void FormationFlightCore::ExtraSecurityCheck(void) {
    if (!vrpnLost && behaviourMode!=BehaviourMode_t::Default) {
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("Optitrack, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void FormationFlightCore::ExtraCheckJoystick(void) {
	/*** Do not use corss, start nor select buttons!! *******/
	// 0: "start"	    // 1: "select"	    // 2: "square"		// 3: "triangle"
    // 4: "circle"	    // 5: "cross";	    // 6: "left 1"	    // 7: "left 2"
    // 8: "left 3"	    // 9: "right 1"	    //10: "right 2"	    //11: "right 3"
    //12: "up"	    	//13: "down"	    //14: "left"	    //15: "right"

	//Se utilizaran los botones superiores.
	if(GetJoystick()->IsButtonPressed(12) && (behaviourMode != BehaviourMode_t::Behaviour1)){ //arriba Home
		cout << "Joy => Se pide acceso al comportamiento 1 \n";
		Ch2Controller1Fnc();
		*ptrInitialLoop = 1;
	    message->SendMessage("Behaviour1");
		}

	if(GetJoystick()->IsButtonPressed(15) && (behaviourMode != BehaviourMode_t::Behaviour2)){ //Derecha  User Positions
		cout << "Joy => Se pide acceso al comportamiento 2\n";
		Ch2Controller2Fnc();
		*ptrInitialLoop = 1;
		message->SendMessage("Behaviour2");
		}
	
	if(GetJoystick()->IsButtonPressed(13) && (behaviourMode != BehaviourMode_t::Behaviour3)){ //Abajo Consensus
		cout << "Joy => Se pide acceso al comportamiento 3\n";
		Ch2Controller3Fnc();
		message->SendMessage("Behaviour3");
		}
	
	if(GetJoystick()->IsButtonPressed(14) && (behaviourMode != BehaviourMode_t::Behaviour4)){
		cout << "Joy => Se pide acceso al comportamiento 4\n";
		Ch2Controller4Fnc();
		message->SendMessage("Behaviour4");
		}
}


void FormationFlightCore::ExtraCheckPushButton(void){
	if(StartBehaviour1Button -> Clicked() && (behaviourMode != BehaviourMode_t::Behaviour1)){
		cout << "Push => Se pide acceso al comportamiento 1\n ";
		Ch2Controller1Fnc();
		message->SendMessage("Behaviour1");

	}
	if(StartBehaviour2Button -> Clicked() && (behaviourMode != BehaviourMode_t::Behaviour2)){
		cout << "Push => Se pide acceso al comportamiento 2\n ";
		Ch2Controller2Fnc();
		message->SendMessage("Behaviour2");

	}
	if(StartBehaviour3Button -> Clicked() && (behaviourMode != BehaviourMode_t::Behaviour3)){
		cout << "Push => Se pide acceso al comportamiento 3 \n";
		Ch2Controller3Fnc();
		message->SendMessage("Behaviour3");
		
	}
	if(StartBehaviour4Button -> Clicked() && (behaviourMode != BehaviourMode_t::Behaviour4)){
		cout << "Push => Se pide acceso al comportamiento 4 \n";
		Ch2Controller4Fnc();
		message->SendMessage("Behaviour4");
		
	}
	
		
}


void FormationFlightCore::LagrangianMatrix(){
}

void FormationFlightCore::StatesDrones(void){
	for( int i = 0; i<3; i++){
		ArrayDrones[i]->GetPosition(PositionAllDrones[i]);
		*(StateX + i) = PositionAllDrones[i].x;
		*(StateY + i) = PositionAllDrones[i].y;
		*(StateZ + i) = PositionAllDrones[i].z;
		cout << "Pos Drone " << i << " : x = " << PositionAllDrones[i].x << " y = " << PositionAllDrones[i].y << " z = " << PositionAllDrones[i].z << endl;
		}
		
	cout << "Los vectores de estados son: " << endl;
	for (int i = 0; i<3 ; i++)
	{
	cout << *(StateX + i) << "     " << *(StateY + i) << "   " << *(StateZ + i) << endl;			
			
		}		
}

// To change to Behaviour1 -> Home
void FormationFlightCore::Ch2Controller1Fnc(void) {
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        behaviourMode=BehaviourMode_t::Behaviour1;
        Thread::Info("Homming ====>> OK\n");
        *ptrInitialLoop = 1;
    } else {
        Thread::Warn("Homming ====>> FAILED\n");
    }

}

// To chage to Behaviour2 -> Position set by user
void FormationFlightCore::Ch2Controller2Fnc(void){
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        behaviourMode=BehaviourMode_t::Behaviour2;
        Thread::Info("User sets position =====>> OK \n");
        *ptrInitialLoop = 1;
    } else {
        Thread::Warn("User sets position =====>> FAILED \n");
    }
}

// To change to Behaviour3 -> Consensus 
void FormationFlightCore::Ch2Controller3Fnc(void) {
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        behaviourMode=BehaviourMode_t::Behaviour3;
        Thread::Info("Consensus ====>> OK\n");
    } else {
        Thread::Warn("Consensus ====>> FAILED \n");
    }

}

//void UAVpffp::StartTracking(void){
void FormationFlightCore::Ch2Controller4Fnc(void){
    if (SetTorqueMode(TorqueMode_t::Custom)) {
        behaviourMode=BehaviourMode_t::Behaviour4;
        Thread::Info("Behaviour4 Not defined ====>> OK \n");
    } else {
        Thread::Warn("Behaviour4 Not defined ====>> FAILED\n");
    }
}


void FormationFlightCore::ComputeCustomTorques(Euler &torques) {
	
	Euler vrpnEuler;
	Quaternion vrpnQuaternion;
    Euler refAngles;
	Euler torque;
    Quaternion refQuaternion;
    Vector3Df referenceOmega;
    Vector3Df uav_pos,uav_vel;
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed,refAngularSpeed;
 
	GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
	Euler ahrsEuler = ahrsQuaternion.ToEuler();
	cout << "Data from IMU: " << ahrsEuler.roll << "   " << ahrsEuler.pitch << "   " << ahrsEuler.yaw << endl;
 
    uavVrpn->GetQuaternion(vrpnQuaternion);
    vrpnEuler = vrpnQuaternion.ToEuler();
    cout << "Data from  Optitrack: " << vrpnEuler.roll << "  " << vrpnEuler.roll << "   " << vrpnEuler.yaw << endl;

	cout << "Yaw from imu: " << ahrsEuler.yaw << "  Yaw from optitrack: " << vrpnEuler.yaw << endl;
    ahrsEuler.yaw = vrpnEuler.yaw;
    cout << "Attitude : " << ahrsEuler.roll << "   " << ahrsEuler.pitch << "   " << ahrsEuler.yaw << endl;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();
    
    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    
    
    GetDefaultReferenceOrientation()->GetQuaternionAndAngularRates(refQuaternion,referenceOmega); // Joystick
    refAngles=refQuaternion.ToEuler();
  
	customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(),Vector3Df(0,0,0));	
	//customReferenceOrientation->SetQuaternionAndAngularRates( refAngles.ToQuaternion(), referenceOmega));	// This is if you want to send the desired angular rate
	
	uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);	
    std::cout<<uav_pos.x<<"   "<<uav_pos.y<<"    "<<uav_pos.z<<std::endl;
    int ModeOperation;
    StatesDrones();	
    
    EstadosQuadMatrix->SetValue(0,0,StateX[0]);
    EstadosQuadMatrix->SetValue(1,0,StateX[1]);
    EstadosQuadMatrix->SetValue(2,0,StateX[2]);
    EstadosQuadMatrix->SetValue(3,0,StateY[0]);
    EstadosQuadMatrix->SetValue(4,0,StateY[1]);
    EstadosQuadMatrix->SetValue(5,0,StateY[2]);
    EstadosQuadMatrix->SetValue(6,0,StateZ[0]);
    EstadosQuadMatrix->SetValue(7,0,StateZ[1]);
    EstadosQuadMatrix->SetValue(8,0,StateZ[2]);

    																							

	/********************************
	 * 			For Homing		 	*
	 * ******************************/
	 
	 if(behaviourMode == BehaviourMode_t::Behaviour1) 
    {
		ModeOperation  = 0;
		cout << "\nHome Behaviour Chosen \n";
	
		Position -> SetValues(customOrientation, uav_pos, uav_vel, ModeOperation, &InitialLoop);
		Position -> Update(GetTime());
		*ptrInitialLoop = 0;
		
		torque.roll 	= Position -> Output( 0 );
		torque.pitch	= Position -> Output( 1 );
		torque.yaw 		= Position -> Output( 2 );
		//cout << "\nTorque.roll : " << torque.roll << " Torque.pitch :" << torque.pitch << " Torque.yaw : " << torque.yaw << endl;
    }


	/**********************************
	 * 		User Position			  *
	 * ********************************/
	 
	 
    if(behaviourMode == BehaviourMode_t::Behaviour2)  // Position Control
    {
		ModeOperation  = 1;
		cout << "\n User pos Behaviour Chosen \n";
	
		Position -> SetValues(customOrientation, uav_pos, uav_vel, ModeOperation, &InitialLoop);
		Position -> Update(GetTime());
		*ptrInitialLoop = 0;
		
		torque.roll 	= Position -> Output( 0 );
		torque.pitch	= Position -> Output( 1 );
		torque.yaw 	    = Position -> Output( 2 );
		cout << "\nTorque.roll : " << torque.roll << " Torque.pitch :" << torque.pitch << " Torque.yaw : " << torque.yaw << endl;
    }
    
    /************************************
     * 		Consensus					*
     * **********************************/
     
    if(behaviourMode == BehaviourMode_t::Behaviour3)
    {
		Formation -> SetValues(customOrientation, &StateX[0], &StateY[0], &IdDrone, uav_vel, &StateZ[0]);
		Formation -> Update(GetTime());
		
		torque.roll 	= Formation -> Output(0);
		torque.pitch	= Formation -> Output(1);
		torque.yaw 		= Formation -> Output(2);
		cout << "\nTorque.roll : " << torque.roll << " Torque.pitch :" << torque.pitch << " Torque.yaw : " << torque.yaw << endl;
		
	}
	
	if(behaviourMode == BehaviourMode_t::Behaviour4)
    {
		cout << "En construccion" << endl;
	}
  
	/********************************************
	 * 		Sending the Output Torques to 		*
	 * 			The		Motors					*
	 * ******************************************/
    torques.roll	= torque.roll;
	torques.pitch	= torque.pitch;
	torques.yaw 	= torque.yaw;
	
	cout << " Torques.roll : " << torques.roll << "   Torques.pitch  : " << torques.pitch << "   Torques.yaw : " << torques.yaw << endl;
	

}

	
	
	
	

	
	
