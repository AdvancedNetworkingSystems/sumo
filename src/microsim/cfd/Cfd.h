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
/// @file    Cfd.h
/// @author  Andrea Stedile
/// @date    Mon, 30 March 2020
///
/****************************************************************************/
#ifndef Cfd_h
#define Cfd_h

#include "CfdVehicle.h"
#include "CfdPlatoon.h"

class Cfd {

public:

    Cfd(const Cfd &) = delete;
    void operator=(const Cfd &) = delete;


    static const Cfd &getInstance() {
        static Cfd instance;
        return instance;
    }

    static double getDragCoefficientReduction(const std::string &vehicleType,
                                              const std::vector<std::string> &precedingVehiclesTypes,
                                              const double interVehicleDistance) {
        return getInstance().getDragCoefficientReduction_Impl(vehicleType,
                                                              precedingVehiclesTypes,
                                                              interVehicleDistance);
    }


private:

    std::set<CfdPlatoon *> platoons;

    Cfd();

    double getDragCoefficientReduction_Impl(const std::string &vehicleType,
                                            const std::vector<std::string> &precedingVehiclesTypes,
                                            const double interVehicleDistance) const;
};


#endif

/****************************************************************************/
