//
// Created by Hamza El-Kebir on 6/14/21.
//

#include <gazebo_crater_catalog_plugin.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CraterCatalogPlugin)

void gazebo::CraterCatalogPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    world_ = model_->GetWorld();
    sdf_ = sdf;

    namespace_.clear();

    if (sdf_->HasElement("robotNamespace")) {
        namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_crater_catalog] Please specify a robotNamespace.\n";
    }

    nodeHandle_ = transport::NodePtr(new transport::Node());
    nodeHandle_->Init(namespace_);

    craterPubTopic_ = "~/" + model_->GetName() + "/crater_catalog";
    craterPub_ = nodeHandle_->Advertise<sensor_msgs::msgs::Crater>(craterPubTopic_, 10);

    vehiclePos_ = Vector3d::Zero;
    closestCrater_.name = "N/A";
    closestCrater_.relPos = Vector3d::Zero;

    initCatalog();
}

void gazebo::CraterCatalogPlugin::Init()
{
    connections_.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&CraterCatalogPlugin::OnUpdate, this)));

    std::cout << "CraterCatalogPlugin::Init" << std::endl;
}

void gazebo::CraterCatalogPlugin::OnUpdate()
{
    updateCatalog();

    sendCraterMessage();
}

void gazebo::CraterCatalogPlugin::initCatalog()
{
    addCraterByName("crater_1");
    addCraterByName("crater_2");
    addCraterByName("crater_3");
    addCraterByName("crater_4");
}

void gazebo::CraterCatalogPlugin::updateCatalog()
{
    // Obtain absolute vehicle position.
    auto pose = model_->WorldPose();
    vehiclePos_ = pose.Pos();

    // Get closest crater characteristics.
    size_t closestCraterIdx = getClosestCraterIdx(vehiclePos_);
    // auto craterPos = getCraterPositionByIndex(closestCraterIdx);
    auto craterName = getCraterNameByIndex(closestCraterIdx);
    auto craterRelPos = getCraterPositionByIndex(closestCraterIdx, vehiclePos_);

    // Update closest crater.
    closestCrater_.name = craterName;
    closestCrater_.relPos = craterRelPos;

    std::cout << "Closest crater is: " << craterName << " @ <" << craterRelPos.X() << ", " << craterRelPos.Y() << ", " << craterRelPos.Z() << ">" << std::endl;

    sendCraterMessage();
}

void gazebo::CraterCatalogPlugin::sendCraterMessage() const
{
    sensor_msgs::msgs::Crater msg;

    msg.set_name(closestCrater_.name);
    msg.set_xrel(closestCrater_.relPos.X());
    msg.set_yrel(closestCrater_.relPos.Y());
    // z location is optional.
    msg.set_zrel(-1);
    // msg.set_zrel(closestCrater_.relPos.Z());

    craterPub_->Publish(msg);
}

bool gazebo::CraterCatalogPlugin::addCraterByName(const std::string &craterName)
{
    physics::ModelPtr modPtr = world_->ModelByName(craterName);
    if (modPtr != nullptr) {
        craters_.push_back(modPtr);
        craterNames_.push_back(craterName);

        return true;
    } else
        return false;
}

const std::string &gazebo::CraterCatalogPlugin::getCraterNameByIndex(const size_t idx) const
{
    assert((idx < craters_.size()) && "Crater index is out of bounds.");

    return craterNames_[idx];
}

gazebo::CraterCatalogPlugin::Vector3d gazebo::CraterCatalogPlugin::getCraterPositionByIndex(const size_t idx,
                                                                                            const Vector3d &origin) const
{
    if (idx >= craters_.size()) {
        assert((idx < craters_.size()) && "Crater index is out of bounds.");

        return Vector3d::Zero;
    } else {
        return craters_[idx]->WorldPose().Pos() - origin;
    }
}

size_t gazebo::CraterCatalogPlugin::getClosestCraterIdx(const Vector3d &pos) const
{
    size_t idx = 0;
    double distMin = INFINITY;
    double distCur;

    for (int i = 0; i < craters_.size(); i++) {
        distCur = pos.Distance(getCraterPositionByIndex(i));
        if (distCur < distMin) {
            idx = i;
            distMin = distCur;
        }
    }

    return idx;
}

gazebo::CraterCatalogPlugin::Vector3d gazebo::CraterCatalogPlugin::getClosestCraterPos(const Vector3d &pos) const
{
    return craters_[getClosestCraterIdx(pos)]->WorldPose().Pos();
}

gazebo::CraterCatalogPlugin::Vector2d gazebo::CraterCatalogPlugin::getClosestCraterPosPlanar(const Vector3d &pos) const
{
    auto relPos = getClosestCraterPos(pos);
    return Vector2d{relPos.X(), relPos.Y()};
}






