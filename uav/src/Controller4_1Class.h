/*!
 * \file Law.h
 * \brief Class defining a PID
 * \author Guillaume Sanahuja, modified by Pedro Flores
 * Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef CONTROLLER4_1CLASS_H
#define CONTROLLER4_1CLASS_H

#include <ControlLaw.h>
#include <Vector3D.h>
#include <Euler.h>
#include <iostream>
#include <TabWidget.h>
#include <Tab.h>

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
    }
}

namespace flair { namespace filter {
    /*! \class Law
    *
    * \brief Class defining a PID
    */
    //class Law : public ControlLaw {
    class Controller4_1Class : public ControlLaw {
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
            Controller4_1Class(gui::TabWidget* tabwidget,std::string name);
            //Law(const gui::LayoutPosition* position,std::string name);

            /*!
            * \brief Destructor
            *
            */
            ~Controller4_1Class();
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
            void SetValues(const core::AhrsData *actualOrientation, const core::Vector3Df uav_pos, const core::Vector3Df uav_vel,int ModeOperation, int *ResetLoop);

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
         
			float NumericalIntegrationError(int InitialLoop, float PositionError);



        private:
            /*!
            * \brief Update using provided datas
            *
            * Reimplemented from IODevice.
            *
            * \param data data from the parent to process
            */


            void UpdateFrom(const core::io_data *data);
			
            flair::gui::DoubleSpinBox *alpha1_roll, *alpha2_roll;
			flair::gui::DoubleSpinBox *alpha1_pitch, *alpha2_pitch;
			flair::gui::DoubleSpinBox *alpha1_yaw, *alpha2_yaw;
			flair::gui::DoubleSpinBox *homePosx, *homePosy, *userPosx, *userPosy;
			flair::gui::DoubleSpinBox *a1, *a2, *a3, *a4, *a5;
			flair::gui::DoubleSpinBox *alpha1_x,*alpha2_x,*alpha3_x;
			flair::gui::DoubleSpinBox *alpha1_y,*alpha2_y,*alpha3_y;
			flair::gui::DoubleSpinBox *alpha1_z,*alpha2_z,*alpha3_z;			
			flair::gui::DoubleSpinBox *Ux_gain,*Uy_gain;
            flair::core::cvmatrix *state;

            flair::core::Time actualTime, previousTime;
            
            float delta_time = 0;
            float delta_error = 0;
            float IntegralError = 0;
            float previousError = 0;
            
            flair::gui::Tab *mainTab;
            flair::gui::Tab *PositionsSingle,*GainsSingle,*GraphesPositionTab;
            flair::gui::TabWidget *tabWidget;
            
                     

    };
} // end namespace filter
} // end namespace flair
#endif // LAW_H
