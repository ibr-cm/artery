/*
 * Artery V2X Simulation Framework
 * Copyright 2018 Christian Hagau
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef MULTIVCLASSMULTITYPEMODULEMAPPER_H
#define MULTIVCLASSMULTITYPEMODULEMAPPER_H

#include "traci/BasicNodeManager.h"
#include "traci/MultiTypeModuleMapper.h"
#include <omnetpp/csimplemodule.h>
#include <tuple>
#include <vector>

namespace omnetpp {
    class cRNG;
    class cXMLElement;
} // namespace omnetpp

namespace traci
{

class MultiVClassMultiTypeModuleMapper : public MultiTypeModuleMapper
{
public:
	void initialize() override;
	omnetpp::cModuleType* vehicle(NodeManager&, const std::string&) override;

private:
	using VehicleType = std::tuple<omnetpp::cModuleType*, double>;
	using VehicleTypeList = std::pair<double, std::vector<VehicleType>>;

	void parseVehicleTypes(const omnetpp::cXMLElement*);

	std::unordered_map<std::string, VehicleTypeList> mVehicleTypes;

	BasicNodeManager* m_manager;
	SubscriptionManager* m_subscription_manager;
};

} // namespace traci

#endif /* MULTIVCLASSMULTITYPEMODULEMAPPER_H */

