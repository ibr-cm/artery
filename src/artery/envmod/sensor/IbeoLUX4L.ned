package artery.envmod.sensor;

simple IbeoLUX4L extends RealisticLidarSensor
{
    fovRange @unit(m) = 50.0 m;
    fovAngle = 110.0; // degree
    rangeAccuracy @unit(m) = 0.1 m; 
    angleResolution = 0.25; // degree
    minimalDistance @unit(m) = 0.0 m;
    attachmentPoint = "FRONT";
}