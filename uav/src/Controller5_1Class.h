/*!
 * \file Law.h
 * \brief Class defining a PID
 * \author Guillaume Sanahuja, modified by Pedro Flores
 * Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef CONTROLLER5_1CLASS_H
#define CONTROLLER5_1CLASS_H

#include <ControlLaw.h>
#include <Vector3D.h>
#include <Euler.h>
#include <iostream>
#include <TabWidget.h>
#include <Tab.h>
#include <CheckBox.h>

namespace flair
{
    namespace core {
        class AhrsData;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class Vector3DSpinBox;
        class Tab;
        class TabWidget;
        class CheckBox;
    }
}

namespace flair { namespace filter {
    /*! \class Law
    *
    * \brief Class defining a PID
    */
    //class Law : public ControlLaw {
    class Controller5_1Class : public ControlLaw {
        public:
            /*!
            * \brief Constructor
            *
            * Construct a PID at given position. \n
            * The PID will automatically be child of position->getLayout() Layout. After calling this function,
            * position will be deleted as it is no longer usefull. \n
            *
            * \param position position to display settings
            * \param name name
            */
            
            //Controller2Class(const gui::LayoutPosition* position,std::string name);
            Controller5_1Class(gui::TabWidget* tabwidget,std::string name);
            //Law(const gui::LayoutPosition* position,std::string name);

            /*!
            * \brief Destructor
            *
            */
            ~Controller5_1Class();
            //~Law();

            /*!
            * \brief Reset integral
            *
            */
            void Reset(void);

            /*!
            * \brief Set input values
            *
            * \param actualOrientation actual orientation
            * \param referenceOrientation reference orientation
            */
            //void SetValues(const core::AhrsData *actualOrientation, const core::Vector3Df uav_pos, const core::Vector3Df uav_vel,int ModeOperation);
            void SetValues(const core::AhrsData *actualOrientation, float *PtrStateX, float *PtrStateY, int *PtrIdDrone, core::Vector3Df uav_vel, float *PtrStateZ);

            /*!
            * \brief Use default plot
            *
            * Plot the output values at position. \n
            * Plot consists of 4 curves: proportional part,
            * derivative part, integral part and
            * the sum of the three. \n
            * After calling this function, position will be deleted as it is no longer usefull. \n
            *
            * \param position position to display plot
            */
            void UseDefaultPlot(const gui::LayoutPosition* position);
         //   void UseDefaultPlot1(const gui::LayoutPosition* position);
         
			



        private:
            /*!
            * \brief Update using provided datas
            *
            * Reimplemented from IODevice.
            *
            * \param data data from the parent to process
            */


            void UpdateFrom(const core::io_data *data);
            
            float *ptrL0;
            float *ptrL1;
            float *ptrL2;
            

			
			flair::gui::DoubleSpinBox *kp_pitch,*kd_pitch;
			flair::gui::DoubleSpinBox *kp_yaw, *kd_yaw;
            flair::gui::DoubleSpinBox *kp_roll, *kd_roll;
       
			flair::gui::DoubleSpinBox *homePosx, *homePosy, *userPosx, *userPosy;
			flair::gui::DoubleSpinBox *a1, *a2, *a3, *a4, *a5, *kp_x, *kd_x, *kp_y, *kd_y;
			flair::gui::DoubleSpinBox *kp_x_consensus, *kd_x_consensus;
			flair::gui::DoubleSpinBox *kp_y_consensus, *kd_y_consensus;
			flair::gui::DoubleSpinBox *ux,*uy;
			flair::gui::DoubleSpinBox *FleetXPos, *FleetYPos, *GainLeaderX, *GainLeaderY;
			flair::gui::DoubleSpinBox *FFlightSelector;
			
			
			flair::gui::CheckBox *Selector00, *Selector01, *Selector02, *Selector03;
            flair::core::cvmatrix *state;
            
            flair::gui::DoubleSpinBox *DA0[3], *DA1[3], *DA2[3];
            float VecDA0[3],VecDA1[3], VecDA2[3], VecLL[3];
            float *ptrDA0 = &VecDA0[0];
            float *ptrDA1 = &VecDA1[0];
            float *ptrDA2 = &VecDA2[0];
            
            flair::gui::DoubleSpinBox *AA0[3], *AA1[3], *AA2[3];
            float VecAA0[3],VecAA1[3], VecAA2[3];
            float *ptrAA0 = &VecAA0[0];
            float *ptrAA1 = &VecAA1[0];
            float *ptrAA2 = &VecAA2[0];
            
            float VecLA0[3],VecLA1[3], VecLA2[3];
            float *ptrLA0 = &VecLA0[0];
            float *ptrLA1 = &VecLA1[0];
            float *ptrLA2 = &VecLA2[0];
            
            flair::gui::DoubleSpinBox *RelDistx[9], *RelDisty[9];
            float VecRelDistx[3], VecRelDisty[3];
            float *ptrVecRelDistx = &VecRelDistx[0];
            float *ptrVecRelDisty = &VecRelDisty[0];
            
            flair::gui::DoubleSpinBox *ConsensusGains[9];
            float VecConsensusGains[3];
            float *ptrConsensusGains = &VecConsensusGains[0];
                  
            flair::gui::DoubleSpinBox *LeaderSelector;


            core::Time previous_time;
            
            float UxLeaderCtrl, UyLeaderCtrl, UxTotalCtrl, UyTotalCtrl;
            
            flair::gui::Tab *mainTab;
            flair::gui::Tab *FormationSelector, *Formation1, *Formation2, *Formation3, *Gains;
            flair::gui::TabWidget *tabWidget;
            
            void DegreeRead( void );
			float DotProduct(float *ptrVector1, float *ptrVector2);
			float FlightFormationProtocol(float *ptrStates, float *ptrVecAdjacency, float *VecGainsProtocol, float *ptrRelativeDistances, int IdDrone);
			float FormationProtocol(float *ptrState, float *ptrLagrangian, float *ptrDistances);
			void AdjacencyRead( void );
			void CreateLaplacian( void );
			void RelativeDistancesVector( int );
			void ConsensusGainsVector( int );
            

    };
} // end namespace filter
} // end namespace flair
#endif // LAW_H
