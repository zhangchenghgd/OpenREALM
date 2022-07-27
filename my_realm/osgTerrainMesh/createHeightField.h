#ifndef OSGTERRAIN_HEIGHT_FIELD_H
#define OSGTERRAIN_HEIGHT_FIELD_H

#include <string>
#include <osg/Node>


osg::Node* createHeightField(const std::string& heightFile, const std::string& texFile);


osg::Node* createHeightField2(const std::string& heightFile, const std::string& texFile);


#endif // !OSGTERRAIN_HEIGHT_FIELD_H
