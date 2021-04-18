#include "traci/MultiVClassMultiTypeModuleMapper.h"
#include "traci/VariableCache.h"
#include <omnetpp/ccomponenttype.h>
#include <omnetpp/cxmlelement.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <inet/common/ModuleAccess.h>

namespace traci
{

Define_Module(MultiVClassMultiTypeModuleMapper)

void MultiVClassMultiTypeModuleMapper::initialize()
{
	auto vehicleTypes = par("vehicleTypes").xmlValue();
	parseVehicleTypes(vehicleTypes);

	BasicNodeManager* manager = inet::getModuleFromPar<BasicNodeManager>(par("basicNodeManager"), this);
	m_subscription_manager = manager->getSubscriptions();
}

void MultiVClassMultiTypeModuleMapper::parseVehicleTypes(const omnetpp::cXMLElement* vehicleTypes)
{
	auto throwError = [](const std::string& msg, const omnetpp::cXMLElement* node) {
		throw omnetpp::cRuntimeError("%s: %s", node->getSourceLocation(), msg.c_str());
	};

	mVehicleTypes.clear();

	for (auto* v : vehicleTypes->getChildrenByTagName("vehicles")) {
		const std::string& vehicleClasses = v->getAttribute("vClass");

		std::vector<std::string> classes;
		boost::split(classes, vehicleClasses, boost::is_any_of(" "));

		for (auto& vehicleClass : classes) {
			double cdfValue = 0.0;

			if (mVehicleTypes.find(vehicleClass) == mVehicleTypes.end()) {
				//new class, allocate vector
				mVehicleTypes[vehicleClass] = std::make_pair(0.0, std::vector<VehicleType>());
			}

			for (omnetpp::cXMLElement* vehicleTag : v->getChildrenByTagName("vehicle")) {
				auto typeString = vehicleTag->getAttribute("type");
				if (!typeString) {
					throwError("missing 'type' attribute in 'vehicle' tag", vehicleTag);
				}
				auto type = omnetpp::cModuleType::get(typeString);

				auto rateAttribute = vehicleTag->getAttribute("rate");
				if (!rateAttribute) {
					throwError("missing 'rate' attribute in 'vehicle' tag", vehicleTag);
				}
				double rate = boost::lexical_cast<double>(rateAttribute);

				cdfValue += rate;
				mVehicleTypes[vehicleClass].second.push_back(std::make_tuple(type, cdfValue));
			}
			if (cdfValue != 1.0) {
				std::cout << std::endl << "vehicle rates do not add up to 1.0 in:" << std::endl
					<< v->getXML() << std::endl;
			}
			mVehicleTypes[vehicleClass].first = cdfValue;
		}
	}
}

omnetpp::cModuleType* MultiVClassMultiTypeModuleMapper::vehicle(NodeManager& manager, const std::string& id)
{
	omnetpp::cModuleType* moduleType = nullptr;

	auto& vehicle = *m_subscription_manager->getVehicleCache(id);
	auto vehicleClass = vehicle.get<libsumo::VAR_VEHICLECLASS>();

	auto vehicleTypeList = mVehicleTypes.find(vehicleClass);
	if (vehicleTypeList != mVehicleTypes.end()) {
		double vehicleCdf = vehicleTypeList->second.first;
		auto vehicleTypes = vehicleTypeList->second.second;
		const double dice = uniform(0.0, vehicleCdf);
		for (VehicleType& vehicleType : vehicleTypes) {
			if (dice < std::get<1>(vehicleType)) {
				moduleType = std::get<0>(vehicleType);
				break;
			}
		}
	} else {
		std::cout << "encountered unknown vehicleClass: " << vehicleClass << std::endl;
	}

	return moduleType;
}

} // namespace traci
