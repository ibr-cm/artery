/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/RealisticFovSensor.h"
#include "artery/application/Middleware.h"
#include <numeric>

using namespace omnetpp;

using LineOfSight = std::array<artery::Position, 2>;
struct Node {std::shared_ptr<artery::EnvironmentModelObject> objectPtr;};
using AdjList = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Node>;
BOOST_GEOMETRY_REGISTER_LINESTRING(LineOfSight)
static const simsignal_t FovCombinedObjectsSignal = cComponent::registerSignal("RealisticFovSensorCombinedObjects");
static const simsignal_t FovVisibleSignal = cComponent::registerSignal("RealisticFovSensorVisibleObjectPoints");
static const simsignal_t FovErasedPointsSignal = cComponent::registerSignal("RealisticFovSensorErasedPoints");
static const simsignal_t FovWrapperOutlineSignal = cComponent::registerSignal("RealisticFovSensorWrapperOutline");
static const simsignal_t FovOffCentreXSignal = cComponent::registerSignal("RealisticFovSensorOffCentreX");
static const simsignal_t FovOffCentreYSignal = cComponent::registerSignal("RealisticFovSensorOffCentreY");
static const simsignal_t FovOffCentreLengthSignal = cComponent::registerSignal("RealisticFovSensorOffCentreLength");
static const simsignal_t FovDimension1Signal = cComponent::registerSignal("RealisticFovSensorDimension1");
static const simsignal_t FovDimension2Signal = cComponent::registerSignal("RealisticFovSensorDimension2");
static const simsignal_t FovRangeErrSignal = cComponent::registerSignal("RealisticFovSensorRangErr");
static const simsignal_t FovAngleErrSignal = cComponent::registerSignal("RealisticFovSensorAngleErr");
static const simsignal_t FovVelocityErrSignal = cComponent::registerSignal("RealisticFovSensorVelocityErr");
static const simsignal_t FovSensorObjectPointDistanceSignal = cComponent::registerSignal("RealisticFovSensorObjectPointDistance");

namespace artery
{

RealisticFovSensor::RealisticFovSensor() :
    mGroupFigure(nullptr), mSensorConeFigure(nullptr), mLinesOfSightFigure(nullptr),
    mObjectsFigure(nullptr), mObstaclesFigure(nullptr)
{
}

void RealisticFovSensor::finish()
{
    if (mGroupFigure) {
        delete mGroupFigure->removeFromParent();
        mGroupFigure = nullptr;
    }
    BaseSensor::finish();
}

void RealisticFovSensor::initialize()
{
    BaseSensor::initialize();

    std::string groupName = getEgoId();
    if (groupName.empty()) {
        const cModule* host = getMiddleware().getIdentity().host;
        assert(host);
        groupName += host->getName();
    }
    groupName += "-" + getSensorName();
    mGroupFigure = new cGroupFigure(groupName.c_str());
    mGlobalEnvironmentModel->getCanvas()->addFigure(mGroupFigure);
    mColor = cFigure::GOOD_DARK_COLORS[getId() % cFigure::NUM_GOOD_DARK_COLORS];

    mFovConfig.egoID = getEgoId();
    mFovConfig.sensorID = getId();
    mFovConfig.sensorPosition = determineSensorPosition(par("attachmentPoint"));

    mFovConfig.fieldOfView.range = par("fovRange").doubleValue() * boost::units::si::meters;
    mFovConfig.fieldOfView.angle = par("fovAngle").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.angleAccuracy = par("angleAccuracy").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.rangeAccuracy = par("rangeAccuracy").doubleValue() * boost::units::si::meters;
    mFovConfig.fieldOfView.velocityAccuracy = par("velocityAccuracy").doubleValue() * boost::units::si::meters_per_second;
    mFovConfig.fieldOfView.angleResolution = par("angleResolution").doubleValue() * boost::units::degree::degrees;
    mFovConfig.fieldOfView.rangeResolution = par("rangeResolution").doubleValue() * boost::units::si::meters;
    mFovConfig.fieldOfView.velocityResolution = par("velocityResolution").doubleValue() * boost::units::si::meters_per_second;
    mFovConfig.fieldOfView.minimalDistance = par("minimalDistance").doubleValue() * boost::units::si::meters;
    mFovConfig.numSegments = par("numSegments");

    initializeVisualization();
}

void RealisticFovSensor::measurement()
{
    Enter_Method("measurement");
    auto detection = detectObjects();
    mLocalEnvironmentModel->complementObjects(detection, *this);
    mLastDetection = std::move(detection);

    emit(FovVisibleSignal,(int)mLastDetection->visiblePoints.size());
    for (const auto& point : mLastDetection->visiblePoints) {
        double xdiff = point.x.value()-mLastDetection->sensorOrigin.x.value();
        double ydiff = point.y.value()-mLastDetection->sensorOrigin.y.value();
        emit(FovSensorObjectPointDistanceSignal, sqrt(pow(xdiff,2)+pow(ydiff,2)));
    }
    for (const auto& objectWrapper : mLastDetection->objectWrapper) {
        emit(FovCombinedObjectsSignal, (int)objectWrapper->getObjects().size());
        emit(FovWrapperOutlineSignal, (int)objectWrapper->getNoisyOutline().size());
        auto dim1 = objectWrapper->getDimension1().value();
        auto dim2 = objectWrapper->getDimension2().value();
        if (dim1 > 0) {
            emit(FovDimension1Signal, dim1);
        }
        if (dim2 > 0) {
            emit(FovDimension2Signal, dim2);
        }
        if (objectWrapper->getCentrePoint().x.value() > 0 || objectWrapper->getCentrePoint().y.value() > 0) {
            for (const auto& obj : objectWrapper->getObjects()) {
                if (!obj.expired()) {
                    auto objShr = obj.lock();
                    double xlen = abs(objShr->getCentrePoint().x.value()-objectWrapper->getCentrePoint().x.value());
                    double ylen = abs(objShr->getCentrePoint().y.value()-objectWrapper->getCentrePoint().y.value());
                    emit(FovOffCentreXSignal, xlen);
                    emit(FovOffCentreYSignal, ylen);
                    emit(FovOffCentreLengthSignal, sqrt(pow(xlen,2)+pow(ylen,2)));
                }
            }
        }
    }
}

SensorDetection RealisticFovSensor::detectObjects()
{
    namespace bg = boost::geometry;
    if (mFovConfig.fieldOfView.range <= 0.0 * boost::units::si::meter) {
        throw std::runtime_error("sensor range is 0 meter or less");
    } else if (mFovConfig.fieldOfView.angle > 360.0 * boost::units::degree::degrees) {
        throw std::runtime_error("sensor opening angle exceeds 360 degree");
    }

    SensorDetection detection = createSensorCone();
    auto preselObjectsInSensorRange = mGlobalEnvironmentModel->preselectObjects(mFovConfig.egoID, detection.sensorCone);
    auto obstacleIntersections = mGlobalEnvironmentModel->preselectObstacles(detection.sensorCone);

    std::unordered_set<std::shared_ptr<EnvironmentModelObstacle>> blockingObstacles;
    auto graph = AdjList();
    std::map<std::shared_ptr<EnvironmentModelObject>, std::tuple<std::vector<Position>, std::vector<Position>, double, boost::graph_traits<AdjList>::vertex_descriptor>> noisyObjects;

    // check if objects in sensor cone are hidden by another object or an obstacle
    for (const auto& object : preselObjectsInSensorRange)
    {
        // filter all outline points which are not in line of sight
        std::vector<Position> visibleObjectPoints = filterLineOfSight(obstacleIntersections, 
                                                        preselObjectsInSensorRange, 
                                                        detection, 
                                                        object->getOutline());

        std::vector<Position> visibleNoisyObjectPoints = applyMeasurementInaccuracy(detection, visibleObjectPoints);

        std::vector<double> noisyVelocities = applyVelocityInaccuracy(visibleNoisyObjectPoints, object->getVehicleData().speed());

        if (!visibleNoisyObjectPoints.empty()) {
            if (detection.objects.empty() || detection.objects.back() != object) {
                detection.objects.push_back(object);
            }

            //calculate average velocity for object
            double averageNoisyVelocity = std::accumulate(noisyVelocities.begin(), noisyVelocities.end(), 0.0) / noisyVelocities.size();

            std::vector<Position> resolutionHull = computeResolutionBounds(detection, visibleNoisyObjectPoints);

            //create vertex for every detected object
            Node vertexNode;
            vertexNode.objectPtr = std::shared_ptr<EnvironmentModelObject>(object);
            auto vertex_descript = boost::add_vertex(vertexNode,graph);

            //store noisy outline, velocity and vertex_descriptor
            noisyObjects.emplace(std::shared_ptr<EnvironmentModelObject>(object), std::make_tuple(visibleNoisyObjectPoints, resolutionHull, averageNoisyVelocity, vertex_descript));
        }
    } // for each object real LOS

    //check if distance between each point is bigger than resolution, connect objects in graph if distance is too small
    //if other points are inside the resolution area, the point is removed
    //graph: https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp
    typedef boost::graph_traits<AdjList>::vertex_descriptor Vertex;
    typedef boost::graph_traits<AdjList>::vertices_size_type VertexIndex; 
    std::vector<VertexIndex> rank(num_vertices(graph));
    std::vector<Vertex> parent(num_vertices(graph));
    typedef VertexIndex* Rank;
    typedef Vertex* Parent;
    boost::disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);
    initialize_incremental_components(graph, ds);
    incremental_components(graph, ds);
    int removedPoints = 0;
    for(auto& noisyObject : noisyObjects)
    {
        auto& object = noisyObject.first;
        auto& noisyObjectPoints = std::get<0>(noisyObject.second);
        auto& resolutionHull = std::get<1>(noisyObject.second);
        auto& objectVelocity = std::get<2>(noisyObject.second);

        if (noisyObjectPoints.empty()) {
            continue;
        }
        
        //check if objectPoints are near other objects
        for (auto& otherNoisyObject : noisyObjects)
        {
            //only check different vehicles
            // auto& objectResolution = otherNoisyObject.first;
            if (otherNoisyObject.first->getExternalId() == object->getExternalId()) {
                continue;
            }

            // check if point of other object's outline is within the resolution box
            auto& otherObjectOutline =  std::get<0>(otherNoisyObject.second);

            std::vector<Position> unselectivePoints;

            for (auto otherObjectOutlinePoint: otherObjectOutline) {
                if (bg::within(otherObjectOutlinePoint, resolutionHull)) {

                    //check if velocity of both objects is distingushable
                    const auto& otherObjectVelocity = std::get<2>(otherNoisyObject.second);
                    bool upperLimit = objectVelocity + (mFovConfig.fieldOfView.velocityResolution.value() / 2.0) < otherObjectVelocity;
                    bool lowerLimit = objectVelocity - (mFovConfig.fieldOfView.velocityResolution.value() / 2.0) > otherObjectVelocity;
                    if (upperLimit || lowerLimit) {
                        EV_INFO << "Found undistinguishable point of " << otherNoisyObject.first->getExternalId() << " within hull of " << noisyObject.first->getExternalId() << std::endl;
                        unselectivePoints.push_back(otherObjectOutlinePoint);

                        auto otherNoisyObjectIterator = std::find(otherObjectOutline.begin(), otherObjectOutline.end(), otherObjectOutlinePoint);
                        auto detectionIterator = std::find(detection.visiblePoints.begin(), detection.visiblePoints.end(), otherObjectOutlinePoint);
                        otherObjectOutline.erase(otherNoisyObjectIterator);
                        if (detectionIterator != detection.visiblePoints.end()) {
                            detection.visiblePoints.erase(detectionIterator);
                        }
                    }
                }
            }

            if (!unselectivePoints.empty()) {
                EV_INFO << "Merging" << noisyObject.first->getExternalId() << " and " << otherNoisyObject.first->getExternalId() << std::endl;
                auto object_vertex_decriptor = std::get<3>(noisyObject.second);
                auto objectResolution_vertex_descriptor =  std::get<3>(otherNoisyObject.second);
                boost::add_edge(object_vertex_decriptor, objectResolution_vertex_descriptor, graph);
                ds.union_set(object_vertex_decriptor, objectResolution_vertex_descriptor);
                // removePoint = true;//delete the current objectpoint
            }
        }
    }
    emit(FovErasedPointsSignal, removedPoints);
    typedef boost::component_index< VertexIndex > Components;
    Components components(parent.begin(), parent.end());
    // Iterate through each disjoint set and access each set member (https://www.boost.org/doc/libs/1_78_0/libs/graph/example/incremental_components.cpp)
    BOOST_FOREACH (VertexIndex disjointSet, components)
    {
        std::vector<std::weak_ptr<EnvironmentModelObject>> combinedObjects;
        std::vector<Position> objectWrapperPoints;
        double sum = 0;
        int size = 0;
        BOOST_FOREACH (VertexIndex disjointSetMember, components[disjointSet])
        {
            auto& ptr = graph[disjointSetMember].objectPtr;
            const auto& noisyObjectPointsMapEntry = noisyObjects.find(ptr);
            if (noisyObjectPointsMapEntry != noisyObjects.end()) {
                const auto& noisyObjectPoints = std::get<0>(noisyObjectPointsMapEntry->second);
                if (!noisyObjectPoints.empty()) {
                    sum += std::get<2>(noisyObjectPointsMapEntry->second);
                    size++;
                    combinedObjects.push_back(std::weak_ptr<EnvironmentModelObject>(ptr));
                    objectWrapperPoints.insert(objectWrapperPoints.end(), noisyObjectPoints.begin(), noisyObjectPoints.end());
                }
            }
        }
        if (size == 0) {
            size = 1;
        }

        // filter all outline points which are not in line of sight
        std::vector<Position> visibleObjectPoints;
        for (auto objectWrapperPoint : objectWrapperPoints)
        {
            LineOfSight lineOfSight;
            lineOfSight[0] = detection.sensorOrigin;
            lineOfSight[1] = objectWrapperPoint;

            if (!bg::crosses(lineOfSight, objectWrapperPoints)) {
                visibleObjectPoints.push_back(objectWrapperPoint);

                if (mDrawLinesOfSight) {
                    detection.visiblePoints.push_back(objectWrapperPoint);
                }
            }
        }

        //create objectWrapper for each disjoint set
        boost::units::quantity<boost::units::si::velocity> averageVelocity = (sum/size) * boost::units::si::meters_per_second;
        detection.objectWrapper.emplace_back(std::make_shared<EnvironmentModelObjectWrapper>(combinedObjects, visibleObjectPoints, averageVelocity));
    }
    detection.obstacles.assign(blockingObstacles.begin(), blockingObstacles.end());
    
    return detection;
}

SensorDetection RealisticFovSensor::createSensorCone() const
{
    SensorDetection detection;
    const auto& egoObj = mGlobalEnvironmentModel->getObject(mFovConfig.egoID);
    if (egoObj) {
        detection.sensorOrigin = egoObj->getAttachmentPoint(mFovConfig.sensorPosition);
        detection.sensorCone = createSensorArc(mFovConfig, *egoObj);
    } else {
        throw std::runtime_error("no object found for ID " + mFovConfig.egoID);
    }
    return detection;
}

void RealisticFovSensor::initializeVisualization()
{
    assert(mGroupFigure);
    mDrawLinesOfSight = par("drawLinesOfSight");
    mDrawObjectWrapper = par("drawObjectWrapper");
    mDrawResolution = par("drawResolution");
    bool drawSensorCone = par("drawSensorCone");
    bool drawObjects = par("drawDetectedObjects");
    bool drawObstacles = par("drawBlockingObstacles");

    if (drawSensorCone && !mSensorConeFigure) {
        mSensorConeFigure = new cPolygonFigure("sensor cone");
        mSensorConeFigure->setLineColor(mColor);
        mGroupFigure->addFigure(mSensorConeFigure);
    } else if (!drawSensorCone && mSensorConeFigure) {
        delete mSensorConeFigure->removeFromParent();
        mSensorConeFigure = nullptr;
    }

    if(mDrawLinesOfSight && !mLinesOfSightFigure) {
        mLinesOfSightFigure = new cGroupFigure("lines of sight");
        mGroupFigure->addFigure(mLinesOfSightFigure);
    } else if (!mDrawLinesOfSight && mLinesOfSightFigure) {
        delete mLinesOfSightFigure->removeFromParent();
        mLinesOfSightFigure = nullptr;
    }

    if (drawObstacles && !mObstaclesFigure) {
        mObstaclesFigure = new cGroupFigure("obstacles");
        mGroupFigure->addFigure(mObstaclesFigure);
    } else if (!drawObstacles && mObstaclesFigure) {
        delete mObstaclesFigure->removeFromParent();
        mObstaclesFigure = nullptr;
    }

    if (drawObjects && !mObjectsFigure) {
        mObjectsFigure = new cGroupFigure("objects");
        mGroupFigure->addFigure(mObjectsFigure);
    } else if (!drawObjects && mObjectsFigure) {
        delete mObjectsFigure->removeFromParent();
        mObjectsFigure = nullptr;
    }
}

const FieldOfView& RealisticFovSensor::getFieldOfView() const
{
    return mFovConfig.fieldOfView;
}

omnetpp::SimTime RealisticFovSensor::getValidityPeriod() const
{
    using namespace omnetpp;
    return SimTime { 200, SIMTIME_MS };
}

SensorPosition RealisticFovSensor::position() const
{
    return mFovConfig.sensorPosition;
}

const std::string& RealisticFovSensor::getSensorCategory() const
{
    static const std::string category = "RealisticFoV";
    return category;
}

const std::string RealisticFovSensor::getSensorName() const
{
    return mFovConfig.sensorName;
}

void RealisticFovSensor::setSensorName(const std::string& name)
{
    mFovConfig.sensorName = name;
}

void RealisticFovSensor::refreshDisplay() const
{
    if (!mLastDetection) {
        return;
    }

    if (mSensorConeFigure) {
        std::vector<cFigure::Point> points;
        for (const auto& position : mLastDetection->sensorCone) {
            points.push_back(cFigure::Point { position.x.value(), position.y.value() });
        }
        mSensorConeFigure->setPoints(points);
    }

    if (mLinesOfSightFigure) {
        // remove previous lines
        while (mLinesOfSightFigure->getNumFigures() > 0) {
            delete mLinesOfSightFigure->removeFigure(0);
        }

        const Position& startPoint = mLastDetection->sensorCone.front();

        for (const Position& endPoint : mLastDetection->visiblePoints) {
            auto line = new cLineFigure();
            line->setLineColor(mColor);
            line->setLineStyle(cFigure::LINE_DASHED);
            line->setStart(cFigure::Point { startPoint.x.value(), startPoint.y.value() });
            line->setEnd(cFigure::Point { endPoint.x.value(), endPoint.y.value() });
            mLinesOfSightFigure->addFigure(line);
        }
    }

    if (mObstaclesFigure) {
        // remove previous obstacles
        while (mObstaclesFigure->getNumFigures() > 0) {
            delete mObstaclesFigure->removeFigure(0);
        }

        for (const auto& obstacle : mLastDetection->obstacles) {
            auto polygon = new cPolygonFigure(obstacle->getObstacleId().c_str());
            polygon->setFilled(true);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::BLUE);
            for (const auto& position : obstacle->getOutline()) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObstaclesFigure->addFigure(polygon);
        }
    }

    if (mObjectsFigure) {
        // remove previous objects
        while (mObjectsFigure->getNumFigures() > 0) {
            delete mObjectsFigure->removeFigure(0);
        }

        for (const auto& object : mLastDetection->objects) {
            auto polygon = new cPolygonFigure(object->getExternalId().c_str());
            polygon->setFilled(false);
            polygon->setFillColor(mColor);
            polygon->setLineColor(cFigure::RED);
            for (const auto& position : object->getOutline()) {
                polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
            }
            mObjectsFigure->addFigure(polygon);          
        }
        if (mDrawObjectWrapper) {
            for (auto& objectWrapper : mLastDetection->objectWrapper) {
                auto polygon = new cPolygonFigure();
                polygon->setFilled(true);
                polygon->setFillColor(mColor);
                // polygon->setFillOpacity(0.5);
                polygon->setLineColor(cFigure::GREEN);
                polygon->setLineWidth(2);
                for (const auto& position : objectWrapper->getNoisyOutline()) {
                    polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
                }
                mObjectsFigure->addFigure(polygon);
            }
        }

        if (mDrawResolution) {
            for (auto& resolution : mLastDetection->objectPointResolutions) {
                auto polygon = new cPolygonFigure();
                polygon->setFilled(false);
                polygon->setLineColor(mColor);
                for (const auto& position : resolution) {
                    polygon->addPoint(cFigure::Point { position.x.value(), position.y.value() });
                }
                mObjectsFigure->addFigure(polygon);
            }
        }
    }
}




} // namespace artery
