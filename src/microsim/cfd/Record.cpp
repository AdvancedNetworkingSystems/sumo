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

#include "Record.h"

std::ostream &operator<<(std::ostream &os, const Record &record) {
    os << "Types: ";
    for (int i = 0; i < record.t.size(); i++) {
        os << record.t[i] << " ";
    }
    os << "\nDistances: ";
    for (int i = 0; i < record.d.size(); i++) {
        os << record.d[i] << " ";
    }
    os << "\nRatios: ";
    for (int i = 0; i < record.r.size(); i++) {
        os << record.r[i] << " ";
    }
    return os;
}

unsigned int Record::N() const {
    return (unsigned int) t.size();
}

bool Record::compatible(const std::vector<std::string> &t) const {
    if (N() != t.size()) {
        return false;
    }
    for (unsigned int i = 0; i < N(); i++) {
        if (this->t[i] != t[i]) {
            return false;
        }
    }
    return true;
}

