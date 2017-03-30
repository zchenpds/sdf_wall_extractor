#include "tinyxml2.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <math.h>

using namespace std;
using namespace tinyxml2;



class Pose{
public:
    double x, y, z;
    double roll, pitch, yaw;
    // Construts from 6 arguments
    Pose(double x=0, double y=0, double z=0, double roll=0, double pitch=0, double yaw=0)
    {}
    // Construts from a string of space-separated numbers
    Pose(const string str)
    {
        istringstream iss(str);
        iss >> x >> y >> z >> roll >> pitch >> yaw;
        cout << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << endl;
    }
};

class Point2{
public:
    double x, y;

    Point2(double x=0, double y=0): x(x), y(y)
    {}

    // transform a point by a pose
    Point2 transform2(const Pose& pose)
    {
        double x2, y2;
        x2 =  x * cos(pose.yaw) + y * sin(pose.yaw) - pose.x;
        y2 = -x * sin(pose.yaw) + y * cos(pose.yaw) - pose.y;
        return Point2(x2, y2);
    }

    friend ostream& operator <<(ostream& os, const Point2& point)
    {
        return os << "(" << point.x << ", " << point.y << ")";
    }
};

class Size{
public:
    double x, y, z;
    Size(const string str)
    {
        istringstream iss(str);
        iss >> x >> y >> z;
        cout << x << ", " << y << ", " << z << endl;
    }

};

class Obstacle{
public:
    //Pose pose1, pose2;
    //Size length;
    Point2 point1, point2;
public:
    Obstacle(XMLElement* eleLinkWall)
    {
        cout << "Wall:" << endl;
        cout << " pose1:";
        Pose pose1(eleLinkWall->FirstChildElement("pose")->GetText());

        // A collision is actually a subwall.
        XMLElement* eleCollision = eleLinkWall->FirstChildElement("collision");
        int collisionCouter = 0;
        while(eleCollision)
        {
            collisionCouter++;
            string strPose, strSize;
            cout << " Collision:\n";
            cout << "  |-- pose2: ";
            strPose = eleCollision->FirstChildElement("pose")->GetText();
            Pose pose2(strPose);

            strSize = eleCollision->FirstChildElement("geometry")
                    ->FirstChildElement("box")
                    ->FirstChildElement("size")->GetText();
            cout << "  |-- size:  ";
            Size wallSize(strSize);
            // Find two end points of a collision
            Point2 point3 = Point2( wallSize.x/2, 0).transform2(pose2).transform2(pose1);
            Point2 point4 = Point2(-wallSize.x/2, 0).transform2(pose2).transform2(pose1);
            // Is this the first collision of the wall?
            if (collisionCouter == 1)
            {
                point1 = point3;
                point2 = point4;
            }
            else
            {
                // Merge the collision with the existing ones;
                addCollision(point3, point4);
            }
            cout << "  |-- Endpoint1: " << point3 << endl;
            cout << "  |-- Endpoint1: " << point4 << endl;
            cout << "  -------------" << endl;
            eleCollision = eleCollision->NextSiblingElement("collision");
        }
        cout << " Endpoint1: " << point1 << endl;
        cout << " Endpoint2: " << point2 << "\n\n";
    }

private:
    void addCollision(Point2& point3, Point2& point4)
    {
        double xmin, ymin, xmax, ymax;
        xmin = xmax = point1.x;
        ymin = ymax = point1.y;

        if (xmin > point2.x) xmin = point2.x;
        if (xmin > point3.x) xmin = point4.x;
        if (xmin > point4.x) xmin = point4.x;

        if (ymin > point2.y) ymin = point2.y;
        if (ymin > point3.y) ymin = point4.y;
        if (ymin > point4.y) ymin = point4.y;

        if (xmax < point2.x) xmax = point2.x;
        if (xmax < point3.x) xmax = point4.x;
        if (xmax < point4.x) xmax = point4.x;

        if (ymax < point2.y) ymax = point2.y;
        if (ymax < point3.y) ymax = point4.y;
        if (ymax < point4.y) ymax = point4.y;

        point1.x = xmin;
        point1.y = ymin;
        point2.x = xmax;
        point2.y = ymax;
    }

};

class SdfExporter{
private:
    XMLDocument sdf;
    vector<Obstacle> walls;
    Pose modelPose;
public:
    SdfExporter(string filePath)
    {
        sdf.LoadFile(filePath.c_str());
        XMLElement *eleLink, *eleModel;
        cout << "Model Pose: ";
        // If the extension is world
        if (filePath.substr(filePath.find_last_of('.')+1) == "world")
            eleModel = sdf.FirstChildElement( "sdf" )->FirstChildElement( "world" )
                    ->FirstChildElement( "model" );
        else if (filePath.substr(filePath.find_last_of('.')+1) == "sdf")
            eleModel = sdf.FirstChildElement( "sdf" )->FirstChildElement( "model" );
        modelPose = Pose(eleModel->FirstChildElement( "pose" )->GetText());
        cout << "\n";

        eleLink = eleModel->FirstChildElement( "link" );
        queue<XMLElement*> eleLinkWalls;

        // Find all link elements with names starting with "Wall"
        while(eleLink)
        {
            string str = eleLink->Attribute("name");
            // cout << str << endl;
            if (str.substr(0, 4) == "Wall")
                eleLinkWalls.push(eleLink);
            eleLink = eleLink->NextSiblingElement("link");
        }

        // Process walls
        while(!eleLinkWalls.empty())
        {

            XMLElement* eleLinkWall = eleLinkWalls.front();
            eleLinkWalls.pop();
            Obstacle wall(eleLinkWall);
            wall.point1 = wall.point1.transform2(modelPose);
            wall.point2 = wall.point2.transform2(modelPose);
            walls.push_back(wall);
        }
        cout << "\nNumber of walls that haven't been processed: " << eleLinkWalls.size() << endl;
    }

    void writeXml(string filePath="")
    {
        if (filePath == "")
            filePath = "sdf.xml";
        ofstream of(filePath.c_str());
        for (Obstacle& wall : walls)
        {
            of << "    <obstacle x1=\"" << wall.point1.x
               << "\" y1=\"" << wall.point1.y
               << "\" x2=\"" << wall.point2.x
               << "\" y2=\"" << wall.point2.y
               << "\"/>\n";
        }

    }
};

int main(int argc, char *argv[])
{
    // SdfExporter se( "../../Downloads/Altorfer_1_v2/model.sdf" );
    SdfExporter se( "sdf\\world_altorfer_v3.world" );
    se.writeXml( "xml\\fahad.xml" );


}
