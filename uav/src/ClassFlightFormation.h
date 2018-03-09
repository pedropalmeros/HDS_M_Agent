//  created:    2015/11/05
//  filename:   SimpleFleet.h
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

#ifndef MULTIAGENT_CORE_H
#define MULTIAGENT_CORE_H

#include <UavStateMachine.h>

namespace flair {
    namespace core {
        class FrameworkManager;
        class UdpSocket;
        class AhrsData;
        class cvmatrix;
    }
    namespace filter {
        //class TrajectoryGenerator2DCircle;
        class Controller4_1Class;
        class Controller5_1Class;		
    }
		namespace meta {
        class MetaVrpnObject;
    }
    namespace gui {
        class DoubleSpinBox;
        class GridLayout;
        class tabWidget;
        class LayoutPosition;
        class Vector3DSpinBox;
        class Tab; 
        class DataPlot1D;
    }
}


class FormationFlightCore : public flair::meta::UavStateMachine {
    public:
        FormationFlightCore(std::string broadcast,flair::sensor::TargetController *controller);
        ~FormationFlightCore();

    private:
        enum class BehaviourMode_t {
            Default,
            Behaviour1, // Home;
            Behaviour2, // Posiciones -> Usuario
            Behaviour3, // Posiciones -> Consenso.
            Behaviour4, // ------------------
        };

//        BehaviourMode_t orientation_state;
        BehaviourMode_t behaviourMode;
        bool vrpnLost;


		// Functions Prototypes 
			void VrpnPositionHold(void);//flight mode
			void StartCircle(void);
			void StopCircle(void);
			void ExtraTakeOff(void);
			void ExtraSecurityCheck(void);
			void ExtraCheckJoystick(void);
			//const flair::core::AhrsData *GetOrientation(void) const;
			//void AltitudeValues(float &z,float &dz) const;
			//void PositionValues(flair::core::Vector2Df &pos_error,flair::core::Vector2Df &vel_error,float &yaw_ref);
			//const flair::core::AhrsData *GetReferenceOrientation(void);
			void SignalEvent(Event_t event);
			void CheckMessages(void);
			void ExtraCheckPushButton(void);
			void Behaviour1(void);
			void Behaviour2(void);
			void Behaviour3(void);
			void Behaviour4(void);
			void StatesDrones(void);
			void LagrangianMatrix(void);
			void Ch2Controller1Fnc(void);		// Home position
			void Ch2Controller2Fnc(void);		// Position Set by User
			void Ch2Controller3Fnc(void);		// Consensus
			void Ch2Controller4Fnc(void);		// Not defined
			void ComputeCustomTorques(flair::core::Euler &torques);
			void VectorRelDist(void);
			void PrintVectorRelDist(void);
			void ReadDoubleSpinBoxes(void);

        
        
        std::string s;
        std::string Name_Drone;
        std::string *ptrName_Drone = &Name_Drone;
        
        flair::gui::PushButton *StartBehaviour1Button, *StartBehaviour2Button, *StartBehaviour3Button, *StartBehaviour4Button;
        flair::core::Vector3Df PositionAllDrones[3];
        flair::core::Vector3Df *PointerPositionAllDrones[3];
        
        flair::core::cvmatrix *EstadosQuadMatrix;
        
        //To separate the states and to generate
        // x = [x0 x1 x2]^T
        // y = [y0 y1 y2]^T
        // z = [z0 z1 z2]z^T
        float StateX[3], StateY[3], StateZ[3];
        float *ptrStateX = &StateX[0];
        float *ptrStateY = &StateY[0];
        float *ptrStateZ = &StateZ[0]; 
                     
        int IdDrone;
        int *ptrIdDrone = &IdDrone;
        
        int InitialLoop;
        int *ptrInitialLoop = &InitialLoop;
               
        float yawHold;
        flair::core::UdpSocket *message;
        flair::core::Time posWait;
        
        flair::gui::Tab *CLateral01, *CLateral02, *CLateral03;
        flair::gui::Tab *PositionTime, *Positionxy;
        
        flair::gui::DataPlot1D *plotx1, *plotx2, *plotx3;
        flair::gui::DataPlot1D *ploty1, *ploty2, *ploty3;

        flair::filter::Controller5_1Class *Formation;
        flair::filter::Controller4_1Class *Position;
        
       // flair::gui::DoubleSpinBox *xCircleCenter,*yCircleCenter,*yDisplacement;
        flair::core::AhrsData *customReferenceOrientation,*customOrientation;
		flair::meta::MetaVrpnObject *uavVrpn, *ArrayDrones[3];
	
};

#endif // MULTIAGENT_CORE_H
