/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#ifndef COMMANDLINEINTERFACE_H
#define COMMANDLINEINTERFACE_H

#include <string>
#include <iostream>
#include <sstream>
#include <math.h>       /* isnan, sqrt */
#include <stdarg.h>
#include <stdlib.h> /*setenv()*/
#include <stdio.h>
#include <sys/poll.h>

#include "crclapp/Globals.h"
#include "crclapp/Crcl2RosMsgApi.h"

#include "rcs/Debug.h"
#include "rcs/RCSThreadTemplate.h"
#include "rcs/RCSMsgQueue.h"

namespace RCS
{


/**
 * @brief The CLI class provides a simple command line interface through
 * a Linux console. It is primitive in that there are no "modern" console features.
 * It does offer an interactive way to test the robot to see if things are working or
 * why they are failing.
 * Commands:
 *
 * degrees - use degrees instead of radians in specifying angles
 * grasp_free_gear - find free gear and grasp
 * place_free_gear")
 * manual - do no do canned gearing demo
 * auto - turn on canned gearing demo
 * radian- use radians instead of degrees in specifying angles
 * slow - scale vel/acc/jerk to be slower
 * medium- normal vel/acc/jerk
 * fast- scale vel/acc/jerk to be faster
 * instances - dump list of gear and part instances plus poses
 * robots -
 * robot  x - use robot x
 * open - open gripper
 * close - close gripper
 * smartclose - use contact sensors to close gripper
 * flip - invert gripper (untested)
 * slower - scale  vel/acc/jerk by a factor of 2x slower
 * faster- scale  vel/acc/jerk by a factor of 2x faster
 * reset -
 * home - move robot to home position
 * safe - move robot to programmed safe position
 * joints 1 2 3 4 5 ... n
 * jog [x | y | z | 1 | 2 | 3 | ... | n]  incr - start jog of axis, use + or - to
 *     jog forward or backward, and cr to quit
 * quit - end app
 * set gripper x  - define gripper width - complicated alogrithm
 * set dwell t - set time to dwell is seconds
 * dwell - dwell
 * pick object - given part pick the part up
 * retract object - move to a retract position from object
 * approach object - move to a approach position from object
 * grasp object - grasp given object
 * moveto object - move robot gripper to given object position (defined offset)
 * ik object - give ik (joint list) of given object
 * where - give robot pose and joint status
 * where object - give pose of object
 * move x,y,z  - move to xyz (assumes last rotation)
 */
class CComandLineInterface : public Thread
{
public:


    int state;
    bool bRunning;

    ///
    /// \brief CLI contrustor for command line interpreter
    ///
    CComandLineInterface();

    ///
    /// \brief InputLoop sees if the terminal has any input terminated by line feed
    /// \return <0 to quit, 0 if ok and continue, >0 if error.
    ///
    //int inputLoop();

    /**
     * @brief inputState return the latest state of the command line.
     * If anythin on the command queue, execute and return the status from
     * the command. If no command return noop.
     * @return status enumeration.
     */
    int inputState();

    void loopCallback(char* line);
    CMessageQueue<std::string> lineq;

    /**
     * @brief ActionCyclic loop for the command line interface.
     * @return 0 if successful
     */
    virtual int action();

    /**
     * @brief init sets up the gnu readline callback and the loop flag.
     */
    virtual void init ( );

    /**
     * @brief Cleanup called when thread is terminating.
     */
    virtual void cleanup ( ){}

    /// \brief InterpretLine given an input line calls appropriate CrclAPI
    /// \param line command line to interpret
    /// \return  0 if ok
    int interpretLine(std::string line);

    /// \brief InterpretMacro
    /// \param macro_name
    /// \return
    ///
    int interpretMacro(std::string macro_name);

    /**
     * @brief interpretFile accepts a file of CLI commands
     * and executes each line
     * @param filename file optionally containing path of file
     * @return 0 if everthing ok
     */
    int interpretFile(std::string filename);


     /// \brief IsDone return if no more interpreting to do (quit commanded)
    /// \return true if done
    ///
    bool isDone() { return _bFlag; }

private:
    bool jog(char c,double amt, int & jnt, int & axis);

    bool _bDegrees;
    bool _bFlag;
    int _ncindex;
    std::vector<std::string> _robotNames;
};

}
#endif // COMMANDLINEINTERFACE_H
