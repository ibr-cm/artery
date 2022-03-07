package artery.envmod.sensor;

simple ContinentalHRL131 extends RealisticLidarSensor
{
    fovRange @unit(m) = 300.0 m;
    fovAngle = 128.0; // degree
    rangeAccuracy @unit(m) = 0.01 m; 
    angleResolution = 0.05; // degree
    minimalDistance @unit(m) = 0.0 m;
    attachmentPoint = "FRONT";
}