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
/// @date    04 September 2020
///
/****************************************************************************/

#ifndef SUMO_RECORD_H
#define SUMO_RECORD_H

#include <ostream>
#include "vector"
#include "string"

class Record {

public:
    unsigned int N() const;

    /**
     * @brief Checks whether this record is compatible with the provided platoon; i.e.,
     * whether they have the same number of vehicles and the vehicles have the same types.
     * @param t The platoon's vehicle types
     * @return 'true' if this record is compatible with the provided platoon, 'false' otherwise
     */
    bool compatible(const std::vector<std::string> &t) const;


public:
    /// @brief The platoon's vehicle types
    std::vector<std::string> t;
    /// @brief Each vehicle's distance from its predecessor
    std::vector<double> d;
    /// @brief Each vehicle's drag coefficient ratio
    std::vector<double> r;
};

std::ostream &operator<<(std::ostream &os, const Record &record);


#endif //SUMO_RECORD_H
