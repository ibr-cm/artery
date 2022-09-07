//
// Artery V2X Simulation Framework
// Copyright 2014-2018 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod.sensor;

moduleinterface RealisticFovSensor
{
    parameters:
        double fovRange @unit(m);
        double fovAngle; // opening angle in degree
        string attachmentPoint;
        int numSegments;

        double angleAccuracy; //degree
        double rangeAccuracy @unit(m);
        double velocityAccuracy @unit(mps);
        double angleResolution; //degree
        double rangeResolution @unit(m);
        double velocityResolution @unit(mps);
        double minimalDistance @unit(m);

        // visualization paramaters
        bool drawSensorCone; // draw sensor cone polygon
        bool drawBlockingObstacles; // draw outline of (static) obstacles blocking a line-of-sight
        bool drawDetectedObjects; // draw outline of (dynamic) detected objects
        bool drawLinesOfSight; // draw lines from ego to every visible corner of a vehicle
        bool drawResolution; // draw box around every object point
        bool drawObjectWrapper; // draw objectWrapper
}
