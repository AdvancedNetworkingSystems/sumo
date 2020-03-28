/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2020 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Slipstream.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    11.06.2013
///
// A device which stands as an implementation slipstream and which outputs movereminder calls
/****************************************************************************/
#ifndef MSDevice_Slipstream_h
#define MSDevice_Slipstream_h


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include "MSVehicleDevice.h"
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOTrafficObject;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_Slipstream
 * @brief A device which collects info on the vehicle trip (mainly on departure and arrival)
 *
 * Each device collects departure time, lane and speed and the same for arrival.
 *
 * @see MSDevice
 */
class MSDevice_Slipstream : public MSVehicleDevice {
public:
    /** @brief Inserts MSDevice_Slipstream-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a slipstream-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief resets counters
    static void cleanup();

public:
    /// @brief Destructor.
    ~MSDevice_Slipstream();



    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     *
     * @return True (always).
     */
    bool notifyMove(SUMOTrafficObject& veh, double oldPos,
                    double newPos, double newSpeed);


    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "slipstream";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;



protected:
    /// @brief the vehicle's current drag coefficient
    double myDragCoefficient;



private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_Slipstream(SUMOVehicle& holder, const std::string& id);



private:
    // private state members of the Slipstream device

    /// @brief the vehicle's predecessors
    std::vector<std::pair<const MSVehicle* const, double>> precedingVehicles;



private:
    /// @brief Invalidated copy constructor.
    MSDevice_Slipstream(const MSDevice_Slipstream&);

    /// @brief Invalidated assignment operator.
    MSDevice_Slipstream& operator=(const MSDevice_Slipstream&);


};


#endif

/****************************************************************************/

