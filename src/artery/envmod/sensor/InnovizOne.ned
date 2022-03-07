package artery.envmod.sensor;

simple InnovizOne extends RealisticLidarSensor
{
    fovRange @unit(m) = 250.0 m;
    fovAngle = 115.0; // degree
    rangeAccuracy @unit(m) = 0.03 m; 
    angleResolution = 0.1; // degree
    minimalDistance @unit(m) = 0.1 m;
    attachmentPoint = "FRONT";
}