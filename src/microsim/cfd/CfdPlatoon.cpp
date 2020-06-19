/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2007-2020 German Aerospace Center (DLR) and others.
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
/// @file    CfdPlatoon.h
/// @author  Andrea Stedile
/// @date    Mon, 31 May 2020
///
/****************************************************************************/


#include "CfdPlatoon.h"

CfdVehicle *CfdPlatoon::getLeader() const {
    if (vehicles.empty())
        return nullptr;
    return vehicles.front();
}

CfdVehicle *CfdPlatoon::getLast() const {
    if (vehicles.empty())
        return nullptr;
    return vehicles.back();
}

CfdVehicle *CfdPlatoon::getMember(int index) const {
    if (index > (int) vehicles.size() - 1)
        return nullptr;
    return vehicles[index];
}

void CfdPlatoon::addVehicle(std::string &vehicleType,
                            double distanceFromPrecedingVehicle = 0,
                            double dragCoefficientReduction = 0.) {
    auto *vehicle = new CfdVehicle(vehicleType);
    if (getLast() != nullptr) {
        getLast()->succeedingVehicle.first = vehicle;
        getLast()->succeedingVehicle.second = distanceFromPrecedingVehicle;
        vehicle->precedingVehicle.first = getLast();
        vehicle->precedingVehicle.second = distanceFromPrecedingVehicle;
    }
    vehicle->dragCoefficientReduction = dragCoefficientReduction;
    vehicles.push_back(vehicle);
}

bool CfdPlatoon::startsWith(std::vector<std::string> &others) const {
    if (others.size() > vehicles.size())
        return false;

    CfdVehicle *vehicle = getLeader();
    for (const auto &vehicleType : others) {
        if (vehicleType != vehicle->vehicleType)
            return false;
        vehicle = vehicle->succeedingVehicle.first;
    }
    return true;
}

