/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/service/EnvmodPrinter.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/traci/VehicleController.h"
#include <boost/units/io.hpp>
#include <omnetpp/clog.h>
#include <fstream>
#include <inet/common/ModuleAccess.h>
#include "artery/utility/IdentityRegistry.h"

namespace artery
{

Define_Module(EnvmodPrinter)

void EnvmodPrinter::initialize()
{
    ItsG5Service::initialize();
    mLocalEnvironmentModel = getFacilities().get_mutable_ptr<LocalEnvironmentModel>();
    //GlobalEnvironmentModel* globalEnv = getFacilities().get_mutable_ptr<GlobalEnvironmentModel>();
    //mVehId = globalEnv->getIdFromRegistry(getFacilities().get_const<traci::VehicleController>().getVehicleId());
    mEgoId = getFacilities().get_const<traci::VehicleController>().getVehicleId();
    mLogData = "measurementTime,id,x,y\n";
}

EnvmodPrinter::~EnvmodPrinter()
{
    std::ofstream myfile;
    myfile.open("results/log_see-through_Veh_" + mEgoId + ".csv");
    myfile << mLogData;
    myfile.close();
}

void EnvmodPrinter::trigger()
{
    Enter_Method("trigger");
    auto& allObjects = mLocalEnvironmentModel->allObjects();

    EV_DETAIL << mEgoId << "--- By category ---" << std::endl;
    printSensorObjectList("Radar Sensor Object List", filterBySensorCategory(allObjects, "Radar"));
    printSensorObjectList("CAM Sensor Object List", filterBySensorCategory(allObjects, "CA"));

    

    EV_DETAIL << mEgoId << "--- By name ---" << std::endl;
    for (auto &sensor: mLocalEnvironmentModel->getSensors()) {
        std::string sensorName = sensor->getSensorName();
        printSensorObjectList(sensorName + " Object List", filterBySensorName(allObjects, sensorName));
    }

    logSensor("SeeThrough");
}

void EnvmodPrinter::logSensor(const std::string &sensorName) {
    auto& allObjects = mLocalEnvironmentModel->allObjects();

    const TrackedObjectsFilterRange& objs = filterBySensorName(allObjects, sensorName);

    for (const auto& obj : objs)
    {
        std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;
        if (obj_ptr.expired()) continue;
        const auto& vd = obj_ptr.lock()->getVehicleData();
        mLogData += vd.updated().str() + "," + obj.first.lock()->getExternalId() + "," + std::to_string(vd.position().x.value()) + "," + std::to_string(vd.position().y.value()) + "\n";
    }

}

void EnvmodPrinter::printSensorObjectList(const std::string& title, const TrackedObjectsFilterRange& objs)
{
    EV_DETAIL << mEgoId << "--- " << title << " (" << boost::size(objs) << " objects) ---" << std::endl;

    for (const auto& obj : objs)
    {
        std::weak_ptr<EnvironmentModelObject> obj_ptr = obj.first;
        if (obj_ptr.expired()) continue; /*< objects remain in tracking briefly after leaving simulation */
        const auto& vd = obj_ptr.lock()->getVehicleData();
        EV_DETAIL
            << "station ID: " << vd.station_id()
            << " lon: " << vd.longitude()
            << " lat: " << vd.latitude()
            << " speed: " << vd.speed()
            << " when: " << vd.updated()
            << " x: " << vd.position().x.value()
            << " y: " << vd.position().y.value()
            << std::endl;
    }
}

} // namespace artery




