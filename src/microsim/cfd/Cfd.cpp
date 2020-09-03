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
#include <iostream>
#include <utils/common/UtilExceptions.h>

#include "Cfd.h"

#define DEBUG_PARSE_FILE
#define DEBUG_ALGORITHM

Cfd::Cfd() {
    std::ifstream file("data.txt");
    if (!file.is_open())
        throw ProcessError("File 'data.txt' not found");

    enum STATE {
        TYPES,
        DISTANCES,
        RATIOS,
        SAVE
    };

    STATE currentState = TYPES;

    Record *record = nullptr;

    std::string currLine;
    while (std::getline(file, currLine)) {
        // Skip this line if it is a comment or empty
        if (currLine.find('#') == 0 || currLine.empty())
            continue;

        std::stringstream ss(currLine);

        if (currentState == TYPES) {
            // Allocate a new record on demand
            record = new Record();
            records.insert(record);

            // The vehicle's type
            std::string t;
            while (ss >> t) {
                record->t.push_back(t);
            }

            if (record->t.size() < 2) {
                throw ProcessError("The platoon requires at least 2 vehicles");
            }
            currentState = DISTANCES;
        } else if (currentState == DISTANCES) {
            // The first vehicle has no predecessors
            record->d.push_back(0.);

            // The vehicle's distance from the predecessor
            double d;
            while (ss >> d) {
                record->d.push_back(d);
            }

            if (record->d.size() != record->t.size()) {
                throw ProcessError(
                        "The platoon requires " + std::to_string(record->t.size() - 1) + " inter-vehicle distances");
            }
            currentState = RATIOS;
        } else {
            // The vehicle's drag coefficient ratio
            double r;
            while (ss >> r) {
                record->r.push_back(r);
            }

            if (record->r.size() != record->t.size()) {
                throw ProcessError(
                        "The platoon requires " + std::to_string(record->t.size()) + " drag coefficient ratios");
            }
            currentState = TYPES;
        }
    }
    file.close();

#ifdef DEBUG_PARSE_FILE
    std::cout << "=== Cfd constructor ===\n";

    for (const Record *r : records) {
        std::cout << "Parsed platoon:\n";
        std::cout << *r << "\n";
    }
#endif
}

double
Cfd::getDragCoefficientRatio_Impl(const std::vector<std::string> &t,
                                  const std::vector<double> &d,
                                  unsigned int i) const {
#ifdef DEBUG_ALGORITHM
    std::cout << "getDragCoefficient" << std::endl;
#endif

    return 1.;
}