/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2020 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSDevice_Slipstream.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    11.06.2013
///
// A device which stands as an implementation slipstream and which outputs movereminder calls
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Slipstream.h"

// Debug switches
#define DEBUG_PRECEDING_VEHICLES

// Constants
const double MAX_TOT_DIST = 120.;
const double MAX_GAP = 20.;




// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Slipstream::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Slipstream Device");
    insertDefaultAssignmentOptions("slipstream", "Slipstream Device", oc);

    oc.doRegister("device.slipstream.parameter", new Option_Float(0.0));
    oc.addDescription("device.slipstream.parameter", "Slipstream Device", "An exemplary parameter which can be used by all instances of the slipstream device");
}


void
MSDevice_Slipstream::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "slipstream", v, false)) {
        // build the device
        // get custom vehicle parameter
        double customParameter2 = -1;
        if (v.getParameter().knowsParameter("slipstream")) {
            try {
                customParameter2 = StringUtils::toDouble(v.getParameter().getParameter("slipstream", "-1"));
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("slipstream", "-1") + "'for vehicle parameter 'slipstream'");
            }

        } else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'slipstream'. Using default of " << customParameter2 << "\n";
        }
        // get custom vType parameter
        double customParameter3 = -1;
        if (v.getVehicleType().getParameter().knowsParameter("slipstream")) {
            try {
                customParameter3 = StringUtils::toDouble(v.getVehicleType().getParameter().getParameter("slipstream", "-1"));
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("slipstream", "-1") + "'for vType parameter 'slipstream'");
            }

        } else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vType parameter 'slipstream'. Using default of " << customParameter3 << "\n";
        }
        MSDevice_Slipstream* device = new MSDevice_Slipstream(v, "slipstream_" + v.getID(),
                                                        oc.getFloat("device.slipstream.parameter"),
                                                        customParameter2,
                                                        customParameter3);
        into.push_back(device);
    }
}

void
MSDevice_Slipstream::cleanup() {
    // cleaning up global state (if any)
}

// ---------------------------------------------------------------------------
// MSDevice_Slipstream-methods
// ---------------------------------------------------------------------------
MSDevice_Slipstream::MSDevice_Slipstream(SUMOVehicle& holder, const std::string& id,
                                   double customValue1, double customValue2, double customValue3) :
        MSVehicleDevice(holder, id),
        myCustomValue1(customValue1),
        myCustomValue2(customValue2),
        myCustomValue3(customValue3),
        myDragCoefficient(-1.){
    std::cout << "initialized device '" << id << "' with myCustomValue1=" << myCustomValue1 << ", myCustomValue2=" << myCustomValue2 << ", myCustomValue3=" << myCustomValue3 << ", myDragCoefficient=" << myDragCoefficient << "\n";
}


MSDevice_Slipstream::~MSDevice_Slipstream() {
}


bool
MSDevice_Slipstream::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    if (!tObject.isVehicle()) {
        return false;
    }
    MSVehicle* veh = static_cast<MSVehicle*>(&tObject);
    std::cout << "device '" << getID() << "' notifyMove: newSpeed=" << newSpeed << "\n";

    // check whether another device is present on the vehicle:
    MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh->getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != nullptr) {
        std::cout << "  veh '" << veh->getID() << " has device '" << otherDevice->getID() << "'\n";
    }

    precedingVehicles.clear();

    double remaining = MAX_TOT_DIST;

    const MSVehicle* last = veh;
    while (remaining > 0.) {
        // See https://github.com/eclipse/sumo/pull/6822
        std::pair<const MSVehicle* const, double> leader = last->getLeader(2 * MAX_GAP);

        if (leader.first == nullptr)
            break;
        if (leader.second > MAX_GAP) {
#ifdef DEBUG_PRECEDING_VEHICLES
            std::cout << leader.first->getID() << "'s gap from " << last->getID() << " exceeds maximum by " << leader.second - MAX_GAP << std::endl;
#endif
            break;
        }
        double gapAndLength = leader.second + leader.first->getLength();
        if (remaining - gapAndLength < 0.) {
#ifdef DEBUG_PRECEDING_VEHICLES
            std::cout << leader.first->getID() << "'s distance from " << veh->getID() << " exceeds maximum by " << gapAndLength - remaining << std::endl;
#endif
            break;
        }

        precedingVehicles.push_back(leader);
        remaining -= gapAndLength;
        last = leader.first;
    }

#ifdef DEBUG_PRECEDING_VEHICLES
    std::cout << "Preceding vehicles: " << std::endl;
    if (!precedingVehicles.empty()) {
        for(auto& item : precedingVehicles)
        {
            std::cout << " [" << item.second << " m] " << item.first->getID();;
        }
        std::cout << std::endl;
    } else
        std::cout << "No preceding vehicles." << std::endl;
#endif

    return true; // keep the device
}


bool
MSDevice_Slipstream::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


bool
MSDevice_Slipstream::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


void
MSDevice_Slipstream::generateOutput(OutputDevice* tripinfoOut) const {
    if (tripinfoOut != nullptr) {
        tripinfoOut->openTag("slipstream_device");
        tripinfoOut->writeAttr("customValue1", toString(myCustomValue1));
        tripinfoOut->writeAttr("customValue2", toString(myCustomValue2));
        tripinfoOut->closeTag();
    }
}

std::string
MSDevice_Slipstream::getParameter(const std::string& key) const {
    if (key == "customValue1") {
        return toString(myCustomValue1);
    } else if (key == "customValue2") {
        return toString(myCustomValue2);
    } else if (key == "meaningOfLife") {
        return "42";
    } else if (key == "dragCoefficient") {
        return toString(myDragCoefficient);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_Slipstream::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == "customValue1") {
        myCustomValue1 = doubleValue;
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


/****************************************************************************/

