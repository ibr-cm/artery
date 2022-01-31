/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Middleware.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/utility/FilterRules.h"
#include <inet/common/ModuleAccess.h>
#include <omnetpp/cxmlelement.h>
#include <utility>

using namespace omnetpp;

namespace artery
{

Define_Module(LocalEnvironmentModel)

static const simsignal_t EnvironmentModelRefreshSignal = cComponent::registerSignal("EnvironmentModel.refresh");

LocalEnvironmentModel::LocalEnvironmentModel() :
    mGlobalEnvironmentModel(nullptr)
{
}

int LocalEnvironmentModel::numInitStages() const
{
    return 2;
}

void LocalEnvironmentModel::initialize(int stage)
{
    if (stage == 0) {
        mGlobalEnvironmentModel = inet::getModuleFromPar<GlobalEnvironmentModel>(par("globalEnvironmentModule"), this);
        mGlobalEnvironmentModel->subscribe(EnvironmentModelRefreshSignal, this);

        auto vehicle = inet::findContainingNode(this);
        mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), vehicle);
        Facilities& fac = mMiddleware->getFacilities();
        fac.register_mutable(mGlobalEnvironmentModel);
        fac.register_mutable(this);
    } else if (stage == 1) {
        initializeSensors();
    }
}

void LocalEnvironmentModel::finish()
{
    mGlobalEnvironmentModel->unsubscribe(EnvironmentModelRefreshSignal, this);
    mObjects.clear();
}

void LocalEnvironmentModel::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == EnvironmentModelRefreshSignal) {
        for (auto* sensor : mSensors) {
            sensor->measurement();
        }
        update();
    }
}

void LocalEnvironmentModel::complementObjects(const SensorDetection& detection, const Sensor& sensor)
{
    
    for (auto& detectedObject : detection.objects) {//check if objects are already tracked by a sensor
        auto foundObject = mObjects.find(detectedObject);
        std::shared_ptr<EnvironmentModelObjectWrapper> pointer = nullptr;
        for (auto objectWr : detection.objectWrapper) {
            for (auto objectWrPtr : objectWr->getObjects()) {
                if (detectedObject == objectWrPtr) {
                    pointer = std::shared_ptr<EnvironmentModelObjectWrapper>(objectWr);
                }
            }
        }
        if (foundObject != mObjects.end()) {
            Tracking& tracking = foundObject->second;
            tracking.tap(&sensor);
            if (sensor.isNoisy()) {//check if noisyOutline is used sensor.hasNoise()
                tracking.addNoiseValue(&sensor, detectedObject->getNoisyOutline());
                tracking.addObjectWrapper(&sensor, pointer);
                for (auto& wrapper : tracking.mWrapperObject) {
                    auto wrapperptr = wrapper.second;
                    std::cout << wrapperptr->getNoisyOutline().size() << " visible Points. Looking at: " << detectedObject->getExternalId()<<"\n";
                    std::cout << "X " << detectedObject->getCentrePoint().x.value() << " Y " <<detectedObject->getCentrePoint().y.value() << "\n";
                    std::cout << "newX " << wrapperptr->getCentrePoint().x.value() << " newY " << wrapperptr->getCentrePoint().y.value() << "\n";
                    std::cout << "L " << detectedObject->getLength().value() << " W " << detectedObject->getWidth().value() <<"\n";
                    std::cout << "newL " << wrapperptr->getLength().value() << " newW " << wrapperptr->getWidth().value() <<"\n";
                }
                std::cout << "\n";
            }
        } else {
            if (sensor.isNoisy()) {//check if noisyOutline is used sensor.hasNoise()
                mObjects.emplace(detectedObject, Tracking { ++mTrackingCounter, &sensor, detectedObject->getNoisyOutline(), pointer});
                auto found = mObjects.find(detectedObject);
                Tracking& tracking = found->second;
                for (auto& wrapper : tracking.mWrapperObject) {
                    auto wrapperptr = wrapper.second;
                    std::cout << wrapperptr->getNoisyOutline().size() <<" visible Points. Looking at: " << detectedObject->getExternalId()<<"\n";
                    std::cout << "X " << detectedObject->getCentrePoint().x.value() << " Y " <<detectedObject->getCentrePoint().y.value() << "\n";
                    std::cout << "newX " << wrapperptr->getCentrePoint().x.value() << " newY " << wrapperptr->getCentrePoint().y.value() << "\n";
                    std::cout << "L " << detectedObject->getLength().value() << " W " << detectedObject->getWidth().value() <<"\n";
                    std::cout << "newL " << wrapperptr->getLength().value() << " newW " << wrapperptr->getWidth().value() <<"\n";
                }
                std::cout << "\n";
            } else {
                mObjects.emplace(detectedObject, Tracking { ++mTrackingCounter, &sensor });
            }
        }
        detectedObject->removeNoisyOutline();//delete noisyOutline from GlobalEnvironmentModel -> next sensor may have no noise
    }
}

void LocalEnvironmentModel::update()
{
    for (auto it = mObjects.begin(); it != mObjects.end();) {
        const Object& object = it->first;
        Tracking& tracking = it->second;
        tracking.update();

        if (object.expired() || tracking.expired()) {
            it = mObjects.erase(it);
        } else {
            ++it;
        }
    }
}

void LocalEnvironmentModel::initializeSensors()
{
    cXMLElement* config = par("sensors").xmlValue();
    for (cXMLElement* sensor_cfg : config->getChildrenByTagName("sensor"))
    {
        cXMLElement* sensor_filters = sensor_cfg->getFirstChildWithTag("filters");
        bool sensor_applicable = true;
        if (sensor_filters) {
            auto identity = mMiddleware->getIdentity();
            FilterRules rules(getRNG(0), identity);
            sensor_applicable = rules.applyFilterConfig(*sensor_filters);
        }

        if (sensor_applicable) {
            cModuleType* module_type = cModuleType::get(sensor_cfg->getAttribute("type"));
            const char* sensor_name = sensor_cfg->getAttribute("name");
            if (!sensor_name || !*sensor_name) {
                sensor_name = module_type->getName();
            }

            cModule* module = module_type->create(sensor_name, this);
            module->finalizeParameters();
            module->buildInside();
            auto sensor = dynamic_cast<artery::Sensor*>(module);

            if (sensor != nullptr) {
                // set sensor name at very early stage so it is available during sensor initialization
                sensor->setSensorName(sensor_name);
            } else {
                throw cRuntimeError("%s is not of type Sensor", module_type->getFullName());
            }

            module->scheduleStart(simTime());
            module->callInitialize();
            mSensors.push_back(sensor);
        }
    }
}

LocalEnvironmentModel::Tracking::Tracking(int id, const Sensor* sensor) : mId(id)
{
    mSensors.emplace(sensor, TrackingTime {});
}

LocalEnvironmentModel::Tracking::Tracking(int id, const Sensor* sensor, std::vector<Position> noisePosition) : mId(id)
{
    mSensors.emplace(sensor, TrackingTime {});
    mNoisyPositions.emplace(sensor, noisePosition);
}

LocalEnvironmentModel::Tracking::Tracking(int id, const Sensor* sensor, std::vector<Position> noisePosition, std::shared_ptr<EnvironmentModelObjectWrapper> wrapperObject) : mId(id)
{
    mSensors.emplace(sensor, TrackingTime {});
    mNoisyPositions.emplace(sensor, noisePosition);
    mWrapperObject.emplace(sensor, wrapperObject);
}

void LocalEnvironmentModel::Tracking::addNoiseValue(const Sensor* sensor, std::vector<Position> noisyPosition)
{
    auto found = mNoisyPositions.find(sensor);
    if (found != mNoisyPositions.end()) {
        found->second = noisyPosition;
    } else {
        mNoisyPositions.emplace(sensor, noisyPosition);
    }
}

void LocalEnvironmentModel::Tracking::addObjectWrapper(const Sensor* sensor, std::shared_ptr<EnvironmentModelObjectWrapper> wrapper)
{
    auto found = mWrapperObject.find(sensor);
    if (found != mWrapperObject.end()) {
        found->second = wrapper;
    } else {
        mWrapperObject.emplace(sensor, wrapper);
    }
}

bool LocalEnvironmentModel::Tracking::expired() const
{
    return mSensors.empty();
}

void LocalEnvironmentModel::Tracking::update()
{
    for (auto it = mSensors.begin(); it != mSensors.end();) {
      const Sensor* sensor = it->first;
      const TrackingTime& tracking = it->second;

      const bool expired = tracking.last() + sensor->getValidityPeriod() < simTime();
      if (expired) {
            it = mSensors.erase(it);
            auto foundNoise = mNoisyPositions.find(sensor);//if sensor entry also has noise data stored, delete it
            if (foundNoise != mNoisyPositions.end()) {
                mNoisyPositions.erase(foundNoise);
            }
            auto foundWrapper = mWrapperObject.find(sensor);//if sensor entry also has wrapperObject stored, delete it TODO: check how to delete if multiple objects are referenced
            if (foundWrapper != mWrapperObject.end()) {
                mWrapperObject.erase(foundWrapper);
            }  
      } else {
          ++it;
      }
    }
}

void LocalEnvironmentModel::Tracking::tap(const Sensor* sensor)
{
    auto found = mSensors.find(sensor);
    if (found != mSensors.end()) {
         TrackingTime& tracking = found->second;
         tracking.tap();
    } else {
         mSensors.emplace(sensor, TrackingTime {});
    }
}


LocalEnvironmentModel::TrackingTime::TrackingTime() :
   mFirst(simTime()), mLast(simTime())
{
}

void LocalEnvironmentModel::TrackingTime::tap()
{
    mLast = simTime();
}


TrackedObjectsFilterRange filterBySensorCategory(const LocalEnvironmentModel::TrackedObjects& all, const std::string& category)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByCategory = [category](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&category](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorCategory() == category;
                });
    };

    auto begin = boost::make_filter_iterator(seenByCategory, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByCategory, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

TrackedObjectsFilterRange filterBySensorName(const LocalEnvironmentModel::TrackedObjects& all, const std::string& name)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByName = [name](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&name](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorName() == name;
                });
    };

    auto begin = boost::make_filter_iterator(seenByName, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByName, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

} // namespace artery
