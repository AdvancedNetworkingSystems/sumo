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

#ifndef SUMO_CFDPLATOON_H
#define SUMO_CFDPLATOON_H

#include <vector>

#include "CfdVehicle.h"

class CfdPlatoon {

public:

    std::vector<CfdVehicle *> vehicles;

    double interVehicleDistance = -1;

public:

    CfdVehicle *getLeader() const;

    CfdVehicle *getLast() const;

    CfdVehicle *getMember(int index) const;

    void addVehicle(std::string &vehicleType,
                    double distanceFromPrecedingVehicle,
                    double dragCoefficientReduction);

    bool startsWith(std::vector<std::string> &others) const;

};

#endif //SUMO_CFDPLATOON_H
