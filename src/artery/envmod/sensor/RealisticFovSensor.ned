//
// Artery V2X Simulation Framework
// Copyright 2014-2018 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.envmod.sensor;

moduleinterface RealisticFovSensor
{
    parameters:
        @signal[RealisticFovSensorVisibleObjectPoints](type=long);
        @statistic[visibleObjectPoints](source=RealisticFovSensorVisibleObjectPoints; record=stats);
        @signal[RealisticFovSensorErasedPoints](type=long);
        @statistic[erasedObjectPoints](source=RealisticFovSensorErasedPoints; record=stats);

        @signal[RealisticFovSensorCombinedObjects](type=long);
        @statistic[combinedObjects](source=RealisticFovSensorCombinedObjects; record=stats, vector);
        @signal[RealisticFovSensorWrapperOutline](type=long);
        @statistic[wrapperOutline](source=RealisticFovSensorWrapperOutline; record=stats, vector);

        @signal[RealisticFovSensorOffCentreX](type=double);
        @statistic[offsetCentreX](source=RealisticFovSensorOffCentreX; record=stats);
        @signal[RealisticFovSensorOffCentreY](type=double);
        @statistic[offsetCentreY](source=RealisticFovSensorOffCentreY; record=stats);
        @signal[RealisticFovSensorOffCentreLength](type=double);
        @statistic[offsetCentreLength](source=RealisticFovSensorOffCentreLength; record=stats; unit=m);
        @signal[RealisticFovSensorDimension1](type=double);
        @statistic[dimension1](source=RealisticFovSensorDimension1; record=stats; unit=m);
        @signal[RealisticFovSensorDimension2](type=double);
        @statistic[dimension2](source=RealisticFovSensorDimension2; record=stats; unit=m);
        @signal[RealisticFovSensorObjectPointDistance](type=double);
        @statistic[objectPointSensorDistance](source=RealisticFovSensorObjectPointDistance; record=stats; unit=m);

        @signal[RealisticFovSensorRangErr](type=double);
        @statistic[rangeError](source=RealisticFovSensorRangErr; record=stats, vector; unit=m);
        @signal[RealisticFovSensorAngleErr](type=double);
        @statistic[angleError](source=RealisticFovSensorAngleErr; record=stats, vector; unit=degree);
        @signal[RealisticFovSensorVelocityErr](type=double);
        @statistic[velocityError](source=RealisticFovSensorVelocityErr; record=stats, vector; unit=mps);


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
