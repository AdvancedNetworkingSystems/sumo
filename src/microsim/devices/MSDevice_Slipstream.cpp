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
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Slipstream.h"
#include <microsim/cfd/Cfd.h>

#include <vector>

// Debug switches
#define DEBUG_INIT
#define DEBUG_NOTIFY_MOVE
#define DEBUG_PRECEDING_VEHICLES
#define DEBUG_SUCCEEDING_VEHICLES
#define DEBUG_DRAG_COEFFICIENT

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
}


void
MSDevice_Slipstream::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "slipstream", v, false)) {
        const SUMOVTypeParameter& typeParams = v.getVehicleType().getParameter();

        // build the device
        // get custom vType parameter
        double myRefDragCoefficient;
        if (! typeParams.knowsParameter("dragCoefficient"))
            throw ProcessError("vehicle '" + v.getID() + "' does not supply vehicle parameter 'dragCoefficient'");
        try {
            myRefDragCoefficient = typeParams.getDouble("dragCoefficient", -1);
        } catch (...) {
            throw ProcessError("Invalid value '" + typeParams.getParameter("dragCoefficient", "-1") + "'for vehicle parameter 'dragCoefficient'");
        }

        MSDevice_Slipstream* device = new MSDevice_Slipstream(v, "slipstream_" + v.getID(), myRefDragCoefficient);
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
MSDevice_Slipstream::MSDevice_Slipstream(SUMOVehicle& holder, const std::string& id, const double myRefDragCoefficient) :
        MSVehicleDevice(holder, id),
        myRefDragCoefficient(myRefDragCoefficient),
        myDragCoefficient(myRefDragCoefficient){
#ifdef DEBUG_INIT
    std::cout << "initialized device '" << id << "' with myDragCoefficient=" << myDragCoefficient << ", myRefDragCoefficient=" << this->myRefDragCoefficient << std::endl;
#endif
}


MSDevice_Slipstream::~MSDevice_Slipstream() {
}


double MSDevice_Slipstream::getDragCoefficient() const {
    return myDragCoefficient;
}


void MSDevice_Slipstream::computeDragCoefficient() {
    std::string vehicleType = getHolder().getVehicleType().getOriginalID();

    std::vector<std::string> precedingVehiclesTypes;
    for (auto& veh : precedingVehicles) {
        precedingVehiclesTypes.push_back(veh->getVehicleType().getOriginalID());
    }

    double interVehicleDistance = 0;
    if (! precedingVehicles.empty()) {
        // Todo: make sure that all the preceding vehicles have (roughly) the same mutual distance
        interVehicleDistance = precedingDistances.front();
    }

    double reduction = Cfd::getDragCoefficientReduction(vehicleType, precedingVehiclesTypes, interVehicleDistance);

    myDragCoefficient = myRefDragCoefficient * (100. + reduction) / 100;

#ifdef DEBUG_DRAG_COEFFICIENT
    std::cout << "Drag coefficient: " << myRefDragCoefficient << " -> " << myDragCoefficient << std::endl;
#endif
}


void MSDevice_Slipstream::computePrecedingVehicles(const MSVehicle* veh) {
    precedingVehicles.clear();
    precedingDistances.clear();

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

        precedingVehicles.push_back(leader.first);
        precedingDistances.push_back(leader.second);
        remaining -= gapAndLength;
        last = leader.first;
    }

#ifdef DEBUG_PRECEDING_VEHICLES
    std::cout << "Preceding vehicles: " << std::endl;
    assert (precedingDistances.size() == precedingVehicles.size());
    if (!precedingVehicles.empty()) {
        for(unsigned long i = 0; i < precedingVehicles.size(); i++) {
            std::cout << "[" << precedingDistances.at(i) << " m] " << precedingVehicles.at(i)->getID() << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "No preceding vehicles." << std::endl;
    }
#endif
}


void MSDevice_Slipstream::computeSucceedingVehicles(const MSVehicle* veh) {
    succeedingVehicles.clear();
    succeedingDistances.clear();

    double remaining = MAX_TOT_DIST;

    const MSVehicle* last = veh;
    while (remaining > 0.) {
        std::pair<const MSVehicle* const, double> follower = last->getLane()->getFollower(last, last->getPositionOnLane(), 2*MAX_GAP, false);

        if (follower.first == nullptr)
            break;
        if (follower.second > MAX_GAP) {
#ifdef DEBUG_SUCCEEDING_VEHICLES
            std::cout << follower.first->getID() << "'s gap from " << last->getID() << " exceeds maximum by " << follower.second - MAX_GAP << std::endl;
#endif
            break;
        }
        double gapAndLength = follower.second + follower.first->getLength();
        if (remaining - gapAndLength < 0.) {
#ifdef DEBUG_SUCCEEDING_VEHICLES
            std::cout << follower.first->getID() << "'s distance from " << veh->getID() << " exceeds maximum by " << gapAndLength - remaining << std::endl;
#endif
            break;
        }

        succeedingVehicles.push_back(follower.first);
        succeedingDistances.push_back(follower.second);
        remaining -= gapAndLength;
        last = follower.first;
    }

#ifdef DEBUG_SUCCEEDING_VEHICLES
    std::cout << "Succeeding vehicles: " << std::endl;
    assert (succeedingVehicles.size() == succeedingDistances.size());
    if (!succeedingVehicles.empty()) {
        for(unsigned long i = 0; i < succeedingVehicles.size(); i++) {
            std::cout << "[" << succeedingDistances.at(i) << " m] " << succeedingVehicles.at(i)->getID() << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "No succeeding vehicles." << std::endl;
    }
#endif
}


bool
MSDevice_Slipstream::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */,
                             double /* newPos */, double /* newSpeed */) {
    if (!tObject.isVehicle()) {
        return false;
    }
    MSVehicle* veh = static_cast<MSVehicle*>(&tObject);

#ifdef DEBUG_NOTIFY_MOVE
    std::cout << "device '" << getID() << "' notifyMove" << std::endl;
#endif

    computePrecedingVehicles(veh);
    computeSucceedingVehicles(veh);

    computeDragCoefficient();

    return true;
}


std::string
MSDevice_Slipstream::getParameter(const std::string& key) const {
    if (key == toString(SUMO_ATTR_REFERENCEDRAGCOEFFIICENT)) {
        return toString(myRefDragCoefficient);
    } else if (key == toString(SUMO_ATTR_ACTUALDRAGOCEFFICIENT)) {
        return toString(myDragCoefficient);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


/****************************************************************************/

