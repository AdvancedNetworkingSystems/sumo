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

#include <vector>
#include <set>
#include "Record.h"

class Cfd {

public:

    Cfd(const Cfd &) = delete;
    void operator=(const Cfd &) = delete;


    static const Cfd &getInstance() {
        static Cfd instance;
        return instance;
    }

    /** @brief Estimates the ratio by which a vehicle's drag coefficient should change due to platooning
     *  @param t The platoon's vehicle types
     *  @param d Each vehicle's distance from its predecessor
     *  @param i Target vehicle's index
     */
    static double getDragCoefficientRatio(const std::vector<std::string> &t, const std::vector<double> &d, unsigned int i) {
        return getInstance().getDragCoefficientRatio_Impl(t, d, i);
    }


private:
    std::set<const Record *> records;

    Cfd();

    double getDragCoefficientRatio_Impl(const std::vector<std::string> &t,
                                        const std::vector<double> &d,
                                        const unsigned int p) const;


    /**
     * @brief Progressively refines the set of records R. See Algorithm 1
     * @param R Set of records to be refined
     * @param d Each vehicle's distance from its predecessor
     * @param p Target vehicle's index
     */
    void
    searchRecords(std::set<const Record *> &R, const std::vector<double> &d, const unsigned int p) const;

    /**
     * @brief Search procedure towards the platoon's head. See Algorithm 2
     * @param R Set of record to be refined
     * @param d Each vehicle's distance from its predecessor
     * @param k Position from which to start searching
     */
    void
    searchToHead(std::set<const Record *> &R, const std::vector<double> &d, unsigned int k) const;

    /**
     * @brief Search procedure towards the platoon's tail. See Algorithm 2
     * @param R Set of record to be refined
     * @param d Each vehicle's distance from its predecessor
     * @param k Position from which to start searching
     */
    void
    searchToTail(std::set<const Record *> &R, const std::vector<double> &d, unsigned int k) const;

};


#endif

/****************************************************************************/
