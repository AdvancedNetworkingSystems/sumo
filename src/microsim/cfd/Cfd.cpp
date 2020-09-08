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
#include <cmath>
#include <limits>
#include <cassert>
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
Cfd::getDragCoefficientRatio_Impl(const std::vector<std::string> &t, const std::vector<double> &d, const unsigned int p) const {
#ifdef DEBUG_ALGORITHM
    std::cout << "getDragCoefficient\n";
#endif

    assert(t.size() >= 2 && t.size() == d.size());
    assert(p >= 0);
    assert(p <= t.size() - 1);

    // Find the compatible records
    std::set<const Record *> R;
    for (const Record *r : records) {
        if (r->compatible(t)) {
            R.insert(r);
        }
    }

    if (R.empty()) {
#ifdef DEBUG_ALGORITHM
        std::cout << "No compatible records.\n";
#endif
        return 1;
    }

    std::set<const Record *> S; // Shorter
    std::set<const Record *> L; // Longer

    if (p == 0) {
        for (const Record *r : R) {
            // Eq. (5)
            if (r->d[1] < d[1]) {
                S.insert(r);
            } // Eq. (6)
            else {
                L.insert(r);
            }
        }
    } else if (p == t.size() - 1) {
        for (const Record *r : R) {
            // Eq. (7)
            if (r->d[p] < d[p]) {
                S.insert(r);
            } // Eq. (8)
            else {
                L.insert(r);
            }
        }
    } else {
        for (const Record *r : R) {
            // Eq. (9)
            if (r->d[p] < d[p] && r->d[p + 1] < d[p + 1]) {
                S.insert(r);
            } // Eq. (10)
            else if (r->d[p] >= d[p] && r->d[p + 1] >= d[p + 1]) {
                L.insert(r);
            }
        }
    }

    searchRecords(S, d, p);
    searchRecords(L, d, p);

    if (S.size() == 1 && L.size() == 0) {
#ifdef DEBUG_ALGORITHM
        std::cout << "|S| = 1 and |L| = 0\n";
#endif
        return 1.;
    }
    if (S.size() == 0 && L.size() == 1) {
#ifdef DEBUG_ALGORITHM
        std::cout << "|S| = 0 and |L| = 1\n";
#endif
        const Record *l = *L.begin();
        return l->r[p];
    }
    if (S.size() == 1 && L.size() == 1) {
#ifdef DEBUG_ALGORITHM
        std::cout << "|S| = 1 and |L| = 1\n";
#endif
        const Record *s = *S.begin();
        const Record *l = *L.begin();

        if (p == 0)
            return (s->r[p] * (l->d[p+1] - d[p+1]) + l->r[p] * (d[p+1] - s->d[p+1])) / (l->d[p+1] - s->d[p+1]);
        else
            // Eq. (16)
            return (s->r[p] * (l->d[p] - d[p]) + l->r[p] * (d[p] - s->d[p])) / (l->d[p] - s->d[p]);
    }
    std::cout << "Something went wrong. |S| = " << S.size() << " and |L| = " << L.size() << "\n";
    return 1.;
}

void
Cfd::searchRecords(std::set<const Record *> &R, const std::vector<double> &d, const unsigned int p) const {
    const unsigned int N = (unsigned int) d.size();
    assert(p >= 0); // First vehicle
    assert(p <= N - 1); // Last vehicle

    if (R.empty())
        return;

    double delta_R_min = std::numeric_limits<double>::max();

    unsigned int k = 1;
    while (R.size() > 1 && p - k >= 0 && p + k <= N - 1) {
        for (auto it = R.begin(); it != R.end();) {
            double eps_f = (*it)->d[p - k + 1] - d[p - k + 1]; // Eq. (13)
            double eps_b = (*it)->d[p + k] - d[p + k]; // Eq. (14)
            double delta_R = pow(eps_f, 2) + pow(eps_b, 2); // Eq. (15)

            if (delta_R > delta_R_min) {
                // Remove this record
                it = R.erase(it);
                continue;
            }
            if (delta_R < delta_R_min) {
                // Remove all the previous records
                it = R.erase(R.begin(), it);
                delta_R_min = delta_R;
                continue;
            }
            it++;
        }
        k++;
    }

    if (R.size() == 1) {
        return;
    }
    if (N == 2) {
        if (p == 0) {
            return searchToTail(R, d, p);
        } else {
            return searchToHead(R, d, p);
        }
    } else {
        if (p - k <= 0) {
            return searchToTail(R, d, p);
        } else if (p + k >= N - 1) {
            return searchToHead(R, d, p);
        }
    }
}

void
Cfd::searchToTail(std::set<const Record *> &R, const std::vector<double> &d, unsigned int k) const {
#ifdef DEBUG_ALGORITHM
    std::cout << "searchToTail\n";
#endif
    const unsigned int N = (unsigned int) d.size();

    double val_min = std::numeric_limits<double>::max();

    while (R.size() > 1 && k <= N - 1) {
        for (auto it = R.begin(); it != R.end();) {
            double val = std::abs((*it)->d[k + 1] - d[k + 1]);

            if (val > val_min) {
                // Remove this record
                it = R.erase(it);
                continue;
            }
            // Remove all the previous records
            if (val < val_min) {
                it = R.erase(R.begin(), it);
                val_min = val;
                continue;
            }
            it++;
        }
        k += 1;
    }
}

void
Cfd::searchToHead(std::set<const Record *> &R, const std::vector<double> &d, unsigned int k) const {
#ifdef DEBUG_ALGORITHM
    std::cout << "searchToHead\n";
#endif
    double val_min = std::numeric_limits<double>::max();

    while (R.size() > 1 && k >= 1) {
        for (auto it = R.begin(); it != R.end();) {
            double val = std::abs((*it)->d[k] - d[k]);

            if (val > val_min) {
                // Remove this record
                it = R.erase(it);
                continue;
            }
            // Remove all the previous records
            else if (val < val_min) {
                it = R.erase(R.begin(), it);
                val_min = val;
                continue;
            }
            it++;
        }
        k -= 1;
    }
}