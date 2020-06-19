/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2020 German Aerospace Center (DLR) and others.
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
/// @file    Cfd.cpp
/// @author  Andrea Stedile
/// @date    Mon, 30 March 2020
///
/****************************************************************************/

#include <fstream>
#include <sstream>
#include <algorithm> // sort

#include "Cfd.h"
#include "CfdVehicle.h"
#include "CfdPlatoon.h"

#define DEBUG_PARSE_FILE
#define DEBUG_ALGORITHM

Cfd::Cfd() {
    std::ifstream file("data.txt");
    if (!file.is_open())
        throw ProcessError("File 'data.txt' not found");

    enum STATE {
        CLASSES,
        DISTANCES,
        REDUCTIONS,
        SAVE
    };
    STATE currentState = CLASSES;

    std::vector<std::string> types;
    std::vector<double> distances;
    std::vector<double> reductions;

    std::string currLine;
    while (std::getline(file, currLine)) {
        // Skip this line if it is a comment or empty
        if (currLine.find('#') == 0 || currLine.empty())
            continue;

        std::stringstream ss(currLine);
        if (currentState == CLASSES) {
            std::string type;
            while (ss >> type) {
                // Todo verify that types exist
                types.push_back(type);
            }
            if (types.size() < 2) {
                throw ProcessError("The provided platoon has less than 2 vehicles");
            }
            currentState = DISTANCES;
        } else if (currentState == DISTANCES) {
            double distance;
            while (ss >> distance) {
                if (distances.size() >= 2) {
                    if (distance != distances.back()) // Thesis simplification: enforce same inter-vehicle distances
                        throw ProcessError("The provided inter-vehicle distances are different");
                }
                distances.push_back(distance);
            }
            currentState = REDUCTIONS;
        } else { // if (currentState == REDUCTIONS) {
            double reduction;
            while (ss >> reduction) {
                reductions.push_back(reduction);
            }

            auto *platoon = new CfdPlatoon();
            for (int i = 0; i < (int) types.size(); i++)
                platoon->addVehicle(types[i], distances[i], reductions[i]);
            platoon->interVehicleDistance = platoon->getLast()->precedingVehicle.second;
            platoons.insert(platoon);

            types.clear();
            distances.clear();
            reductions.clear();
            currentState = CLASSES;
        }
    }
    file.close();

#ifdef DEBUG_PARSE_FILE
    std::cout << "=== Cfd constructor ===" << std::endl;

    for (CfdPlatoon *platoon : platoons) {
        std::cout << "Parsed platoon: " << std::endl;
        CfdVehicle *vehicle = platoon->getLeader();
        while (vehicle != nullptr) {
            std::cout << "* Vehicle:" << std::endl;
            std::cout << "    Class: " << vehicle->vehicleType << std::endl;
            if (vehicle->precedingVehicle.first != nullptr) {
                std::cout << "    Pred: "
                          << vehicle->precedingVehicle.first->vehicleType
                          << " [" << vehicle->precedingVehicle.second << " m]" << std::endl;
            }
            if (vehicle->succeedingVehicle.first != nullptr) {
                std::cout << "    Succ: "
                          << vehicle->succeedingVehicle.first->vehicleType
                          << " [" << vehicle->succeedingVehicle.second << " m]" << std::endl;
            }
            std::cout << "    Reduction: " << vehicle->dragCoefficientReduction << "%" << std::endl;

            vehicle = vehicle->succeedingVehicle.first;
        }
    }
#endif
}

// Todo: this is a very primitive implementation, and it does not consider succeeding vehicles
double
Cfd::getDragCoefficientReduction_Impl(const std::string &vehicleType,
                                      const std::vector<std::string> &precedingVehiclesTypes,
                                      const double interVehicleDistance) const {
#ifdef DEBUG_ALGORITHM
    std::cout << "getDragCoefficient" << std::endl;
    std::cout << "\tVehicle type: " << vehicleType << std::endl;
    if (!precedingVehiclesTypes.empty()) {
        std::cout << "\tPreceding vehicles types: ";
        for (auto &type: precedingVehiclesTypes) {
            std::cout << type << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "\tInter-vehicle distance: " << interVehicleDistance << std::endl;
#endif

    std::vector<std::string> vehicleTypes(precedingVehiclesTypes.rbegin(), precedingVehiclesTypes.rend());
    vehicleTypes.push_back(vehicleType);

    std::vector<CfdPlatoon *> haveCompatibleGeometry;
    for (auto &platoon : platoons) {
        if (platoon->startsWith(vehicleTypes)) {
            haveCompatibleGeometry.push_back(platoon);
        }
    }

    std::vector<CfdPlatoon *> haveLowerIVDistance;
    std::vector<CfdPlatoon *> haveGreaterIVDistance;
    for (auto &p : haveCompatibleGeometry) {
        if (p->interVehicleDistance < interVehicleDistance) {
            haveLowerIVDistance.push_back(p);
        } else {
            haveGreaterIVDistance.push_back(p);
        }
    }
    std::sort(haveLowerIVDistance.begin(), haveLowerIVDistance.end(),
              [](const CfdPlatoon *p1, const CfdPlatoon *p2) {
                  return p1->interVehicleDistance < p2->interVehicleDistance;
              });
    std::sort(haveGreaterIVDistance.begin(), haveGreaterIVDistance.end(),
              [](const CfdPlatoon *p1, const CfdPlatoon *p2) {
                  return p1->interVehicleDistance < p2->interVehicleDistance;
              });


    if (haveLowerIVDistance.empty()) {
#ifdef DEBUG_ALGORITHM
        std::cout
                << "There is no CFD data about platoons that have a compatible geometry and have a lower inter-vehicle distance" << std::endl;
#endif
        return 0.;
    }
    if (haveGreaterIVDistance.empty()) {
#ifdef DEBUG_ALGORITHM
        std::cout
                << "There is no CFD data about platoons that have a compatible geometry and have a greater inter-vehicle distance" << std::endl;
#endif
        return 0.;
    }
#ifdef DEBUG_ALGORITHM
    std::cout << "Platoons that have a compatible geometry and have a lower inter-vehicle distance:" << std::endl;
    for (auto &p: haveLowerIVDistance) {
        for (auto &v : p->vehicles) {
            if (v != p->getLeader()) {
                std::cout << "[ " << v->precedingVehicle.second << " m ] ";
            }
            std::cout << v->vehicleType << " ( " << v->dragCoefficientReduction << "% ) ";
        }
        std::cout << std::endl;
    }
    std::cout << "Platoons that have a compatible geometry and have a greater inter-vehicle distance:" << std::endl;
    for (auto &p: haveGreaterIVDistance) {
        for (auto &v : p->vehicles) {
            if (v != p->getLeader()) {
                std::cout << "[ " << v->precedingVehicle.second << " m ] ";
            }
            std::cout << v->vehicleType << " ( " << v->dragCoefficientReduction << "% ) ";
        }
        std::cout << std::endl;
    }
#endif

    int pos = (int) vehicleTypes.size() - 1;

    double x1 = haveGreaterIVDistance.front()->interVehicleDistance;
    double y1 = haveGreaterIVDistance.front()->getMember(pos)->dragCoefficientReduction;

    double x2 = interVehicleDistance;

    double x3 = haveLowerIVDistance.back()->interVehicleDistance;
    double y3 = haveLowerIVDistance.back()->getMember(pos)->dragCoefficientReduction;

    double y2 = ((x2 - x1) * (y3 - y1)) / (x3 - x1) + y1;

#ifdef DEBUG_ALGORITHM
    std::cout << x1 << "m [" << x2 << "m] " << x3 << "m" << std::endl;
    std::cout << y1 << "% [" << y2 << "%] " << y3 << "%" << std::endl;
#endif

    return y2;
}