/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#ifndef CSVREADER_H
#define CSVREADER_H

#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Example usage
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// int main()
// {
//     // Creating an object of CSVWriter
//     CSVReader reader("example.csv");

//     // Get the data from CSV File
//     std::vector<std::vector<std::string> > dataList = reader.getData();

//     // Print the content of row by row on screen
//     for(std::vector<std::string> vec : dataList)
//     {
//         for(std::string data : vec)
//         {
//             std::cout<<data << " , ";
//         }
//         std::cout<<std::endl;
//     }
//     return 0;
// }
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

namespace gc
{

/*
 * A class to read data from a csv file.
 */
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    CSVReader( std::string filename, std::string delm = "," ) :
            fileName( filename),
            delimeter( delm )
    { }

    // Function to fetch data from a CSV File
    std::vector< std::vector< std::string > > getData()
    {
        std::ifstream file( fileName );

        std::vector< std::vector< std::string > > dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while ( getline( file, line ) )
        {
            if ( !line.empty() )
            {
                if ( '#' != line[ 0 ] )
                {
                    std::vector< std::string > vec;
                    boost::algorithm::split( vec, line, boost::is_any_of( delimeter ) );
                    dataList.push_back( vec );
                }
            }
        }
        // Close the File
        file.close();

        return dataList;
    }
};

} // namespace gc

#endif // CSVREADER_H
