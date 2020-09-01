//
//  main.cpp
//  GCodeParser
//
//  Created by Visa Harvey on 27.8.2020.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "gparse.h"

using namespace std;

int main(int argc, const char * argv[]) {

    std::ifstream infile("/Users/visa/Documents/Metropolia/Year3/Project/GCodeParser/GCodeParser/gcode01.txt");
    
    std::string line;
     
    uint32_t badcount = 0;
    
    char input[] = "G1 X93 Y57.36 A1\r\n";
    
    command parsed = GCodeParser(input);
    
    while (std::getline(infile, line))
    {
        command parsed = GCodeParser(const_cast<char*>(line.c_str()));
        
        if (!line.empty() && line[line.length()-1] == '\r') {
            line.erase(line.length()-1);
        }
        cout << line << endl; //" -> ";
        if ( parsed.cmd == bad)
        {
            cout << "Bad" << endl;
            ++badcount;
        }
        else
        {
            cout << "OK " << endl;
            switch ( parsed.cmd ) // here is where data could be passed into next layer after parsing.
            {
                case none : cout << "Empty line" << endl; break;
                case init: cout<< "Init" << endl; break;
                case limit: cout<< "limit" << endl; break;
                case savepen: cout<< "savepen( u" <<parsed.penstore.up <<  ", d" << parsed.penstore.down << " )" << endl; break;
                case setpen: cout<< "setpen( " << parsed.pen.pos << " )" << endl; break;
                case savestepper: cout<< "savestepper" << endl; break;
                case setlaser: cout<< "setlaser( " << parsed.laser.power << " )"<< endl; break;
                case origin: cout<< "origin" << endl; break;
                case goxy: cout<< "goxy( x" << parsed.pos.x/100.0 << ", y" << parsed.pos.y/100.0 << ", abs=" << parsed.pos.abs << ")" << endl; break;
                default:
                    cout << "Bad" << endl;
            }
            cout << endl;
        }
    }

    cout << badcount << " erroneous inputs found" << endl;
    
    return 0;
}
