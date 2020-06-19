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
/// @file    CfdVehicle.h
/// @author  Andrea Stedile
/// @date    Mon, 31 May 2020
///
/****************************************************************************/

#ifndef SUMO_CFDVEHICLE_H
#define SUMO_CFDVEHICLE_H

#include <utils/common/SUMOVehicleClass.h>

class CfdVehicle {

public:

    std::string vehicleType;
    std::pair<CfdVehicle *, double> precedingVehicle;
    std::pair<CfdVehicle *, double> succeedingVehicle;
    double dragCoefficientReduction;

public:

    CfdVehicle(std::string &vehicleType);

};

#endif //SUMO_CFDVEHICLE_H
