//
// Artery V2X Simulation Framework
// Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod.sensor;

simple RealisticLidarSensor like RealisticFovSensor
{
    parameters:
        double fovRange @unit(m) = default(80.0 m);
        double fovAngle = default(60.0); // degree
        string attachmentPoint = default("FRONT");
        int numSegments = default(1);
        bool doLineOfSightCheck = default(true);

        double angleAccuracy = default(0.0); // degree
        double rangeAccuracy @unit(m) = default(0.0 m); 
        double velocityAccuracy @unit(mps) = default(0.0 mps);
        double angleResolution = default(0.0); // degree
        double rangeResolution @unit(m) = default(0.0 m); 
        double velocityResolution @unit(mps) = default(0.0 mps);
        double minimalDistance @unit(m) = default(0.0 m);

        bool drawSensorCone = default(false);
        bool drawDetectedObjects = default(false);
        bool drawBlockingObstacles = default(false);
        bool drawLinesOfSight = default(false);
        bool drawObjectWrapper = default(false);
        bool drawResolution = default(false);
}
