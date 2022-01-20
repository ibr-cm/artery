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
        if (foundObject != mObjects.end()) {
            Tracking& tracking = foundObject->second;
            tracking.tap(&sensor);
            if (!(detectedObject->getNoisyOutline().empty())) {//check if noisyOutline is used sensor.hasNoise()
                tracking.addNoiseValue(&sensor, detectedObject->getNoisyOutline());
                //std::cout << "not empty sensor: " << sensor.getSensorName() << " detectedObject: " << detectedObject->getExternalId() <<"\n";
                /*auto &stuff = tracking.mNoisyPositions;
                std::cout << "Old Tracking added by: " << sensor.getSensorName() << " NoisyPositions size: " << stuff.size();
                std::cout << " Object: " << detectedObject->getExternalId() << " Position:\n";
                for (auto itOb = detectedObject->getOutline().begin(); itOb != detectedObject->getOutline().end(); itOb++) {
                    std::cout << " x: " << itOb->x.value();
                    std::cout << " y: " << itOb->y.value();
                }     
                std::cout << "\n";
                for (auto itstuff = stuff.begin(); itstuff != stuff.end(); itstuff++) {
                    std::cout << " sensor: " << itstuff->first->getSensorName() << "\n";
                    for (auto ittstuff = itstuff->second.begin(); ittstuff != itstuff->second.end() ; ittstuff++) {
                        std::cout << " x: " << ittstuff->x.value() << " y: " << ittstuff->y.value();
                    }
                }
                std::cout << "\n";*/
            }
        } else {
            if (!(detectedObject->getNoisyOutline().empty())) {//check if noisyOutline is used sensor.hasNoise()
                mObjects.emplace(detectedObject, Tracking { ++mTrackingCounter, &sensor, detectedObject->getNoisyOutline() });  
                //std::cout << "NEW not empty sensor: " << sensor.getSensorName() << "\n";     
                /*auto foundObjectTWO = mObjects.find(detectedObject);
                Tracking& nostuff = foundObjectTWO->second;
                auto &stuff = nostuff.mNoisyPositions;
                std::cout << "New Tracking added by: " << sensor.getSensorName() << " NoisyPositions size: " << stuff.size();
                std::cout << " Object: " << detectedObject->getExternalId() << " Position:\n";
                for (auto itOb = detectedObject->getOutline().begin(); itOb != detectedObject->getOutline().end(); itOb++) {
                    std::cout << " x: " << itOb->x.value();
                    std::cout << " y: " << itOb->y.value();
                }     
                std::cout << "\n";
                for (auto itstuff = stuff.begin(); itstuff != stuff.end(); itstuff++) {
                    std::cout << " sensor: " << itstuff->first->getSensorName() << "\n";
                    for (auto ittstuff = itstuff->second.begin(); ittstuff != itstuff->second.end() ; ittstuff++) {
                        std::cout << " x: " << ittstuff->x.value() << " y: " << ittstuff->y.value();
                    }
                }
                std::cout << "\n";*/
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

void LocalEnvironmentModel::Tracking::addNoiseValue(const Sensor* sensor, std::vector<Position> noisyPosition)
{
    auto found = mNoisyPositions.find(sensor);
    if (found != mNoisyPositions.end()) {
        found->second = noisyPosition;
    } else {
        mNoisyPositions.emplace(sensor, noisyPosition);
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
            auto found = mNoisyPositions.find(sensor);//if sensor entry also has noise data stored, delete it
            if (found != mNoisyPositions.end()) {
                mNoisyPositions.erase(found);
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
