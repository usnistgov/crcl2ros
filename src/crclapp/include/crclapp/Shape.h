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

#ifndef Shape_Model_H
#define Shape_Model_H

// C++
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <list>
#include <cstring>
#include <memory>
#include <mutex>
#include <functional>

// ROS
#include <tf/tf.h>

// Application specific
#include <crclapp/Globals.h>
#include <rcs/Conversions.h>


namespace WorldModel {

extern std::mutex shapemutex;

/**
 * @brief The CShape struct is the basic definition of a shape in the world model.
 * In this case its a gear, kit or vessel. Kits and vessels
 * have slots that hold the gears. The slots are defined for holders offset from the
 * centroid of the holder. Each shape has a unique name, a type defined as a small, medium
 * or large gear and vessel holders, and kits that hold combinations of gear types.
 */
struct CShape
{
public:

    CShape(){// ouch
        this->_bEmpty=true;
    }

    CShape(std::string name,
          std::string type,
          tf::Pose pose,
          CShape * parent=NULL);

    std::string name() const;
    void setName(const std::string &name);

    std::string type() const;
    void setType(const std::string &type);

    tf::Pose centroid();
    void setCentroid(const tf::Pose &centroid);

    ///////////////////////////////

    std::string _name;
    std::string _type;  // gear, vessel, kit for now
    CShape * _parent;
    tf::Pose _dimensions;  // size of xyz in meters
    tf::Pose _location;  // location of xyz bottom of object.
    bool _bEmpty;
    std::vector< CShape > _contains;  // slots in tray
    std::map<std::string, std::string> _attributes;
    std::map<std::string, std::map<std::string, std::string>> _properties ;

    bool isKit()
    {
        if(_name.find("kit") != std::string::npos)
            return true;
        return false;
    }
    bool isVessel() // tray
    {
        // kit also has vessel in its name
        if(_name.find("gear_vessel") != std::string::npos)
            return true;
        return false;
    }
    bool isGear()
    {
        if(_name.find("part") != std::string::npos)
            return true;
        return false;
    }
    bool isSkuPart()
    {
        // If not a sku skip
        if(_name.find("sku") != std::string::npos)
            return true;
        return false;
    }
    bool inMyWorld();

};

/**
 * @brief The CShapes struct base class is a std vector of shapes.
 * The list of shapes is defined by the static definition initDefinition.
 * Once setup, the variety of shapes exist and one can use findDefintion
 * to find matching Shape definition.
 */
struct CShapes : std::vector<CShape>
{
    static void initDefinitions();
    CShape *findDefinition(std::string type);
};

/**
 * @brief The CInstances struct contains all the instances
 * of the shapes in the world.
 */
struct CInstances : std::vector<CShape>
{
    /**
     * @brief firstOrderLogic updates the list of instances
     * to reflect any changes in scene. in particular, gears
     * that have moved from one tray to another, open one slot
     * in one tray, and use the slot in the other tray.
     * @return 0 if worked.
     */
    int firstOrderLogic();
    /**
     * @brief findFreeGearKitSlot cycle through all instances - look for gear part, see if in slot,
     * if so return false; othwerwise nothing found in slot, return true.
     * @param gear  - gear to store
     * @param pos_slot - position of slot to store gear
     * @param myparts parts robot can reach within list of all parts
     * @return number of open slots. Zero if no open slots.
     */
    int findFreeGearKitSlot(CShape* gear,
                            tf::Pose &pos_slot,
                            std::vector<std::string> myparts);


    /**
     * @brief storeInstance save a model instance and centroid position.
     * @param name of part
     * @param centroid 6d position - xyz + orientation
     * @param mesh file name
     * @param scale is the scaling factor to convert to meters
     */
    void storeInstance(std::string name,
                       tf::Pose centroid,
                       std::string meshfile,
                       tf::Vector3 scale);
    /**
     * @brief findInstance find shape definition given name
     * @param name string name
     * @return shape definition
     */
    CShape * findInstance(std::string name);

    /**
     * @brief findFreeGear searches gear vessels for free gear
     * @param myparts list of myparts of all models in world
     * @param now_pose current pose to find closest free
     * @return shape that is closet and free
     */
    CShape * findFreeGear(std::vector<std::string> myparts,
                          tf::Pose now_pose);

    /**
     * @brief getDefinition given a model type find definition.
     * @param type model type
     * @return shape definition
     */
    CShape * getDefinition(std::string type);

    CShape * findSlot(CShape * instance, std::string name);

    /**
     * @brief findEmptyContainerSlot given current vector of shape locations
     * @param kit   parameter out of  kit with empty slot
     * @param slot parameter out of found empty slot in kit
     * @return true if found empty slot in kit
     */
    bool findEmptyContainerSlot(std::vector<CShape>&,
                                CShape * kit,
                                CShape * slot);

    /**
     * @brief isInsideContainerSlot determines if there is part inside container
     * @param container  vessel or kit with slots
     * @param part shape description
     * @return
     */
    bool isInsideContainerSlot(CShape * container,
                               CShape * part);

    /**
     * @brief insideContainer  find a gear IN a vessel NOT inside a kit.
     * Find a gear vessel, then find it vessel definition,
     * make sure containuer type matches part type, and gear inside vessel container.
     * @param now_instances current position of all model instances
     * @param part
     * @param types_container describes container type
     * @param not_types_container not types
     * @return shape or null if inside container.
     */
    CShape * insideContainer(std::vector<CShape> & now_instances,
                                       CShape * part,
                                       std::vector<std::string> types_container,
                                       std::vector<std::string> not_types_container);

    /**
     * @brief dumpLocations prepare string describing all instance locations.
     * @return string
     */
    std::string dumpLocations();
    std::string dumpLocations(std::function<tf::Pose(tf::Pose)> Arg2);

    /**
     * @brief dumpInstances dumps instance location as well as first order
     * slot name, location and state and if containing part, part name
     * @return string containing list of gears, kits, and vessels
     */
    std::string dumpInferences();

};

/**
 * @brief definitions list of all shape definitions possible in model.
 */
extern CShapes definitions;



};
#endif
