//
// Created by Hamza El-Kebir on 6/14/21.
//

#ifndef _GAZEBO_CRATER_CATALOG_PLUGIN_HH_
#define _GAZEBO_CRATER_CATALOG_PLUGIN_HH_

#include <mutex>
#include <string>
#include <vector>
#include <iostream>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <ignition/math.hh>

#include "Crater.pb.h"

namespace gazebo {

    /**
     * @brief Struct that contains accessible closest crater data.
     *
     * @details Contains the name and relative position of the nearest crater. The crater depth can be inferred from the
     * crater name provided the same naming convention is adhered to as in the sample world.
     *
     * Note that in the default configuration, the z-value of \c relaPos will be -1.
     */
    struct ClosestCrater {
        std::string name;
        ignition::math::Vector3<double> relPos;
    };

    /**
     * @brief Class that contains logic for realistic closest crater reporting.
     */
    class GAZEBO_VISIBLE CraterCatalogPlugin : public ModelPlugin {
    public:
        CraterCatalogPlugin() {};

        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

        virtual void Init();

        bool addCraterByName(const std::string &craterName);

        using Vector3d = ignition::math::Vector3<double>;
        using Vector2d = ignition::math::Vector2<double>;


    protected:
        void OnUpdate();

        /**
         * @brief Initializes crater list.
         */
        void initCatalog();

        /**
         * @brief Fetches current vehicle position and computes closest crater characteristics.
         */
        void updateCatalog();

        /**
         * @brief Sends Crater message to ~/lander/crater_catalog
         */
        void sendCraterMessage() const;

        /**
         * @brief Gets crater name from \c craterNames_ by index.
         *
         * @details Raises runtime exceptions on range transgression.
         *
         * @param idx Crater index.
         * @return Crater name.
         */
        const std::string &getCraterNameByIndex(const size_t idx) const;

        /**
         * @brief Get crater position by index.
         *
         * @details Raises runtime exceptions on range transgression.
         *
         * @param idx Crater index.
         * @param origin Origin relative to which position is computed, default is 0.
         * @return Crater position in world frame.
         */
        Vector3d getCraterPositionByIndex(const size_t idx,
                                          const Vector3d &origin = Vector3d::Zero) const;

        /**
         * @brief Computes closest crater index.
         *
         * @param pos Vehicle position.
         * @return Closest crater index.
         */
        size_t getClosestCraterIdx(const Vector3d &pos) const;

        /**
         * @brief Obtains closest crater position.
         *
         * @param pos Vehicle position.
         * @return Closest crater position in world frame.
         */
        Vector3d getClosestCraterPos(const Vector3d &pos) const;

        /**
         * @brief Obtains closest crater position in plane.
         *
         * @param pos Vehicle position.
         * @return Closest crater position in plane in world coordinates.
         */
        Vector2d getClosestCraterPosPlanar(const Vector3d &pos) const;

        physics::ModelPtr model_;
        sdf::ElementPtr sdf_;
        physics::WorldPtr world_; //! Pointer to world object.

        std::string namespace_;
        std::vector<event::ConnectionPtr> connections_;
        transport::NodePtr nodeHandle_;

        std::vector<physics::ModelPtr> craters_; //! Vector of registered crater model pointers.
        std::vector<std::string> craterNames_; //! Vector of registered crater names.

        transport::PublisherPtr craterPub_;
        std::string craterPubTopic_;

        ignition::math::Vector3<double> vehiclePos_; //! Current vehicle position.

        ClosestCrater closestCrater_; //! Struct containing closest crater characteristics.
    };

}

#endif //_GAZEBO_CRATER_CATALOG_PLUGIN_HH_
