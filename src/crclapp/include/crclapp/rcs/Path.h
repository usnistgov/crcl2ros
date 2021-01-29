#ifndef PATH_H
#define PATH_H

#include <string>
#include <vector>
#include <stdlib.h>     /* stat */

#include <rcs/File.h>

class Path
{
public:

    static std::string path(std::string filepath)
    {
        return filepath.substr(0, filepath.find_last_of('/') + 1);
    }

    static std::string find(std::vector<std::string> folders, std::string filename)
    {

        // add empty string in case filename is a full path
        folders.insert(folders.begin(), "");

        for(size_t i=0; i< folders.size(); i++)
        {
            struct stat buffer;
            std::string filepath=folders[i]+filename;
            if (stat (filepath.c_str(), &buffer) == 0)
            {
                return filepath;
            }

        }
        return "";
    }
};

#endif // PATH_H
